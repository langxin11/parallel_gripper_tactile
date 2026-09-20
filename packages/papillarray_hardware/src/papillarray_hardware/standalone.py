"""独立触觉记录模式的稳定接触门禁与逐 pillar 摩擦估计策略。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

from .acquisition import TactileSnapshot, TactileWorker
from .pillar_friction import PillarFrictionConfig, PillarFrictionEstimator


@dataclass(frozen=True, slots=True)
class StandaloneSlipConfig:
    """不依赖夹爪控制器的逐点自主估计与可选原厂对照配置。"""

    max_duration_s: float | None = None
    native_slip_enabled: bool = False
    own_friction_enabled: bool = True
    stable_duration_s: float = 0.5
    max_force_rate_n_s: float = 0.5
    # 单 pillar 的 Fz 误差尺度为 0.05 N；进入／退出门槛分别保留约 3 倍／2 倍裕量。
    contact_on_n: float = 0.15
    contact_off_n: float = 0.1
    contact_loss_confirm_s: float = 0.03
    sample_timeout_s: float = 0.05
    confirmation_timeout_s: float = 0.2
    min_estimates_per_side: int = 1
    stop_when_estimates_ready: bool = False
    own_friction: PillarFrictionConfig = field(default_factory=PillarFrictionConfig)

    def __post_init__(self) -> None:
        """拒绝无效时限、矛盾阈值与非整数触点数量。"""
        for name in (
            "stable_duration_s",
            "max_force_rate_n_s",
            "contact_on_n",
            "contact_off_n",
            "contact_loss_confirm_s",
            "sample_timeout_s",
            "confirmation_timeout_s",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} 必须为正有限数")
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} 必须为正有限数")
        if self.max_duration_s is not None and (
            isinstance(self.max_duration_s, bool)
            or not isinstance(self.max_duration_s, (int, float))
            or not math.isfinite(self.max_duration_s)
            or self.max_duration_s <= 0
        ):
            raise ValueError("max_duration_s 必须为正有限数或 None")
        if (
            isinstance(self.min_estimates_per_side, bool)
            or not isinstance(self.min_estimates_per_side, int)
            or self.min_estimates_per_side <= 0
        ):
            raise ValueError("min_estimates_per_side 必须为正整数")
        if not isinstance(self.stop_when_estimates_ready, bool):
            raise ValueError("stop_when_estimates_ready 必须为布尔值")
        if not isinstance(self.native_slip_enabled, bool):
            raise ValueError("native_slip_enabled 必须为布尔值")
        if not isinstance(self.own_friction_enabled, bool):
            raise ValueError("own_friction_enabled 必须为布尔值")
        if not self.native_slip_enabled and not self.own_friction_enabled:
            raise ValueError("至少必须启用一条摩擦估计路径")
        if not isinstance(self.own_friction, PillarFrictionConfig):
            raise ValueError("own_friction 必须为 PillarFrictionConfig")
        if self.contact_off_n >= self.contact_on_n:
            raise ValueError("contact_off_n 必须小于 contact_on_n")
        if self.max_duration_s is not None and self.confirmation_timeout_s >= self.max_duration_s:
            raise ValueError("confirmation_timeout_s 必须小于 max_duration_s")


class StandaloneSlipSession:
    """只依据触觉稳定性启动一次估计，并保存逐触点滑移证据。"""

    def __init__(self, config: StandaloneSlipConfig) -> None:
        """初始化接触滞回、稳定窗口与逐触点状态。"""
        self.config = config
        self._previous: TactileSnapshot | None = None
        self._mask: tuple[tuple[bool, ...], ...] = ()
        self._reference_mask: tuple[tuple[bool, ...], ...] = ()
        self._force_history: deque[tuple[float, float, float]] = deque()
        self._started = False
        self._last_status: tuple[int, str, str] | None = None
        self._previous_states: tuple[tuple[int, ...], ...] = ()
        self._estimates: dict[tuple[int, int], float] = {}
        self._own_estimator = PillarFrictionEstimator(config.own_friction)
        self._own_estimates: dict[tuple[int, int], tuple[float, float]] = {}
        self._cross_validated: set[tuple[int, int]] = set()
        self._own_active = False
        self._contact_lost_since_s: float | None = None

    def update(
        self, sample: TactileSnapshot, *, now_s: float, worker: TactileWorker
    ) -> list[dict[str, object]]:
        """消费一个新快照；稳定接触后启动，并按配置或安全条件停止。"""
        events: list[dict[str, object]] = []
        status = worker.native_slip_status
        signature = (status.session_id, status.phase, status.reason)
        if self.config.native_slip_enabled and signature != self._last_status:
            events.append(
                {
                    "event": "native_slip_state",
                    "session_id": status.session_id,
                    "phase": status.phase,
                    "reason": status.reason,
                    "host_monotonic_s": now_s,
                }
            )
            self._last_status = signature
        if self.config.native_slip_enabled and status.phase == "failed":
            raise RuntimeError(f"原厂滑移服务失败：{status.reason}")

        previous = self._previous
        fresh = 0 <= now_s - sample.received_at_s <= self.config.sample_timeout_s
        new = previous is None or sample.timestamp_us > previous.timestamp_us
        if not fresh or not new:
            if not fresh:
                self._force_history.clear()
                events.extend(self._stop_own_estimator(sample, now_s, "stale_sample"))
                if self.config.native_slip_enabled and status.phase in {
                    "queued",
                    "starting",
                    "active",
                }:
                    worker.stop_native_slip("stale_sample")
            return events

        gap = previous is not None and (
            sample.received_at_s - previous.received_at_s > self.config.sample_timeout_s
            or (sample.timestamp_us - previous.timestamp_us) * 1e-6 > self.config.sample_timeout_s
            or bool(sample.counter_gap)
        )
        mask = self._contact_mask(sample)
        changed = bool(self._mask) and mask != self._mask
        valid_contact = len(mask) == 2 and all(
            sum(side) >= self.config.min_estimates_per_side for side in mask
        )
        self._mask = mask

        running = self.config.native_slip_enabled and status.phase in {
            "queued",
            "starting",
            "active",
            "stopping",
        }
        if gap:
            self._force_history.clear()
            self._contact_lost_since_s = None
            events.extend(self._stop_own_estimator(sample, now_s, "sample_gap"))
            if running:
                worker.stop_native_slip("sample_gap")
        elif not valid_contact:
            self._force_history.clear()
            if self._own_active:
                if self._contact_lost_since_s is None:
                    self._contact_lost_since_s = sample.received_at_s
                elif (
                    sample.received_at_s - self._contact_lost_since_s
                    >= self.config.contact_loss_confirm_s
                ):
                    events.extend(self._stop_own_estimator(sample, now_s, "contact_lost"))
            if running:
                worker.stop_native_slip("contact_lost")
        else:
            self._contact_lost_since_s = None
            if changed and not running:
                self._force_history.clear()
            elif not self._started and not running:
                self._append_force_history(sample)
                if self._force_window_is_stable():
                    if self.config.native_slip_enabled:
                        worker.request_native_slip(
                            1,
                            max_duration_s=self.config.max_duration_s,
                            confirmation_timeout_s=self.config.confirmation_timeout_s,
                        )
                    self._started = True
                    self._force_history.clear()
                    self._reference_mask = mask
                    self._previous_states = sample.native_pillar_states
                    if self.config.own_friction_enabled:
                        self._own_estimator.start(mask)
                        self._own_active = True
                        events.append(
                            {
                                "event": "own_friction_state",
                                "phase": "active",
                                "reason": "stable_contact",
                                "device_timestamp_us": sample.timestamp_us,
                                "packet_counter": sample.packet_counter,
                                "received_at_s": sample.received_at_s,
                                "host_monotonic_s": now_s,
                            }
                        )
            elif not running:
                self._force_history.clear()

        if (
            self.config.native_slip_enabled
            and status.phase == "active"
            and not gap
            and valid_contact
            and self._matching_active_sample(sample, status.session_id)
        ):
            evidence_mask = self._reference_mask or mask
            events.extend(self._collect_estimates(sample, evidence_mask, now_s))
            if self.config.stop_when_estimates_ready and all(
                sum(side == key[0] for key in self._estimates) >= self.config.min_estimates_per_side
                for side in range(2)
            ):
                worker.stop_native_slip("estimates_ready")
            if self._topology_matches(sample.native_pillar_states, evidence_mask):
                self._previous_states = sample.native_pillar_states

        if self._own_active and not gap and valid_contact:
            for estimate in self._own_estimator.update(sample, mask):
                key = (estimate.side, estimate.pillar_id)
                self._own_estimates[key] = (estimate.raw_mu, estimate.conservative_mu)
                events.append(
                    {
                        "event": "own_friction_estimate",
                        "side": "left" if estimate.side == 0 else "right",
                        "pillar_id": estimate.pillar_id,
                        "raw_mu": estimate.raw_mu,
                        "conservative_mu": estimate.conservative_mu,
                        "normal_force_n": estimate.normal_force_n,
                        "shear_force_n": estimate.shear_force_n,
                        "device_timestamp_us": estimate.device_timestamp_us,
                        "packet_counter": estimate.packet_counter,
                        "received_at_s": sample.received_at_s,
                        "host_monotonic_s": now_s,
                    }
                )
        events.extend(self._cross_validation_events(sample, now_s))

        self._previous = sample
        return events

    def _stop_own_estimator(
        self, sample: TactileSnapshot, now_s: float, reason: str
    ) -> list[dict[str, object]]:
        """停止活动的自主估计，并返回一次可审计的状态事件。"""
        if not self._own_active:
            return []
        self._own_estimator.stop()
        self._own_active = False
        return [
            {
                "event": "own_friction_state",
                "phase": "stopped",
                "reason": reason,
                "device_timestamp_us": sample.timestamp_us,
                "packet_counter": sample.packet_counter,
                "received_at_s": sample.received_at_s,
                "host_monotonic_s": now_s,
            }
        ]

    def _cross_validation_events(
        self, sample: TactileSnapshot, now_s: float
    ) -> list[dict[str, object]]:
        """同一 pillar 的两条估计均有效时记录一次原始系数差异。"""
        events: list[dict[str, object]] = []
        matched = (self._estimates.keys() & self._own_estimates.keys()) - self._cross_validated
        for key in sorted(matched):
            native_mu = self._estimates[key]
            own_raw_mu, own_conservative_mu = self._own_estimates[key]
            absolute_error = abs(own_raw_mu - native_mu)
            events.append(
                {
                    "event": "friction_cross_validation",
                    "side": "left" if key[0] == 0 else "right",
                    "pillar_id": key[1],
                    "own_raw_mu": own_raw_mu,
                    "own_conservative_mu": own_conservative_mu,
                    "native_mu": native_mu,
                    "absolute_error": absolute_error,
                    "relative_error_to_native": absolute_error / native_mu,
                    "device_timestamp_us": sample.timestamp_us,
                    "packet_counter": sample.packet_counter,
                    "received_at_s": sample.received_at_s,
                    "host_monotonic_s": now_s,
                }
            )
            self._cross_validated.add(key)
        return events

    def _contact_mask(self, sample: TactileSnapshot) -> tuple[tuple[bool, ...], ...]:
        """用逐触点法向力滞回生成双侧接触集合。"""
        forces = (sample.left_taxel_forces_n, sample.right_taxel_forces_n)
        return tuple(
            tuple(
                math.isfinite(row[2])
                and row[2]
                >= (
                    self.config.contact_off_n
                    if len(self._mask) == 2
                    and index < len(self._mask[side])
                    and self._mask[side][index]
                    else self.config.contact_on_n
                )
                for index, row in enumerate(side_forces)
            )
            for side, side_forces in enumerate(forces)
        )

    def _append_force_history(self, sample: TactileSnapshot) -> None:
        """保留恰好覆盖稳定时长的低通双侧法向力窗口。"""
        self._force_history.append(
            (sample.received_at_s, sample.left_force_n, sample.right_force_n)
        )
        cutoff = sample.received_at_s - self.config.stable_duration_s
        while len(self._force_history) > 1 and self._force_history[1][0] <= cutoff:
            self._force_history.popleft()

    def _force_window_is_stable(self) -> bool:
        """用窗口极差抑制逐包噪声，限制稳定期间的最大法向漂移。"""
        if len(self._force_history) < 2:
            return False
        duration_s = self._force_history[-1][0] - self._force_history[0][0]
        if duration_s + 1e-12 < self.config.stable_duration_s:
            return False
        allowed_range_n = self.config.max_force_rate_n_s * duration_s
        for side in (1, 2):
            values = [item[side] for item in self._force_history]
            if not all(math.isfinite(value) for value in values):
                return False
            if max(values) - min(values) > allowed_range_n:
                return False
        return True

    @staticmethod
    def _topology_matches(
        values: tuple[tuple[object, ...], ...], mask: tuple[tuple[bool, ...], ...]
    ) -> bool:
        """检查一个原厂逐触点数组与当前拓扑完全一致。"""
        return len(values) == len(mask) == 2 and all(
            len(side_values) == len(side_mask)
            for side_values, side_mask in zip(values, mask, strict=True)
        )

    def _matching_active_sample(self, sample: TactileSnapshot, session_id: int) -> bool:
        """只接受采集线程标记为本次双侧活动会话的同包数据。"""
        return (
            sample.native_session_id == session_id
            and sample.native_session_phase == "active"
            and sample.native_slip_active == (True, True)
        )

    def _collect_estimates(
        self,
        sample: TactileSnapshot,
        mask: tuple[tuple[bool, ...], ...],
        now_s: float,
    ) -> list[dict[str, object]]:
        """把新的 SLIPPED 转换与正有限摩擦估计记录成独立事件。"""
        states = sample.native_pillar_states
        estimates = sample.native_pillar_friction
        if not self._topology_matches(states, mask) or not self._topology_matches(estimates, mask):
            return []
        events: list[dict[str, object]] = []
        for side in range(2):
            for pillar_id, state in enumerate(states[side]):
                old_state = (
                    self._previous_states[side][pillar_id]
                    if len(self._previous_states) == 2
                    and pillar_id < len(self._previous_states[side])
                    else None
                )
                mu = estimates[side][pillar_id]
                if (
                    mask[side][pillar_id]
                    and state == 3
                    and old_state != 3
                    and mu is not None
                    and math.isfinite(mu)
                    and mu > 0
                ):
                    self._estimates[(side, pillar_id)] = mu
                    events.append(
                        {
                            "event": "native_slip_estimate",
                            "session_id": sample.native_session_id,
                            "side": "left" if side == 0 else "right",
                            "pillar_id": pillar_id,
                            "native_mu": mu,
                            "device_timestamp_us": sample.timestamp_us,
                            "packet_counter": sample.packet_counter,
                            "received_at_s": sample.received_at_s,
                            "host_monotonic_s": now_s,
                        }
                    )
        return events
