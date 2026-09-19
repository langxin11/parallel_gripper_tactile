"""稳定接触门禁与原厂滑移短时旁路辨识会话。"""

from __future__ import annotations

from dataclasses import dataclass, fields
import math

from papillarray_hardware import TactileSnapshot, TactileWorker


@dataclass(frozen=True, slots=True)
class NativeSlipConfig:
    """原厂旁路辨识参数；最长租期必须显式指定，其他值为待标定起点。

    Args:
        max_duration_s: 从发送启动命令起计的最长租期，单位 s。
        stable_duration_s: 启动前连续稳定窗口，单位 s。
        force_tolerance_n: 每侧法向力与目标的允许偏差，单位 N。
        force_rate_n_s: 两侧法向力及目标允许的变化率，单位 N/s。
        contact_on_n: 单触点接触进入阈值，单位 N。
        contact_off_n: 单触点接触退出阈值，单位 N。
        sample_timeout_s: 观测失鲜或间断阈值，单位 s。
        confirmation_timeout_s: 启停反馈确认时限，单位 s。
        cooldown_s: 确认停止后的最短重启间隔，单位 s。
        estimate_expiry_s: 已确认估计的有效期，单位 s。
        min_estimates_per_side: 提前结束所需的每侧有效单点估计数。
        max_sessions: 一次抓取允许的会话数，默认只辨识一次。
    """

    max_duration_s: float
    stable_duration_s: float = 0.3
    force_tolerance_n: float = 0.15
    force_rate_n_s: float = 0.5
    contact_on_n: float = 0.05
    contact_off_n: float = 0.025
    sample_timeout_s: float = 0.05
    confirmation_timeout_s: float = 0.2
    cooldown_s: float = 2.0
    estimate_expiry_s: float = 5.0
    min_estimates_per_side: int = 1
    max_sessions: int = 1

    def __post_init__(self) -> None:
        """拒绝无限会话和无效门槛。"""
        for item in fields(self):
            value = getattr(self, item.name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"native_slip.{item.name} 必须为正有限数")
        for value in (self.min_estimates_per_side, self.max_sessions):
            if not isinstance(value, int):
                raise ValueError("原厂辨识触点数和会话数必须为整数")
        if self.contact_off_n >= self.contact_on_n:
            raise ValueError("单触点退出阈值必须小于进入阈值")
        if self.confirmation_timeout_s >= self.max_duration_s:
            raise ValueError("原厂确认时限必须小于最大运行时长")


class NativeSlipSession:
    """按新触觉样本门控原厂服务；估计仅记录，不自动接管目标力。"""

    def __init__(self, config: NativeSlipConfig) -> None:
        """初始化会话编号、稳定窗口和分触点结果。"""
        self.config = config
        self.session_id = 0
        self._previous: TactileSnapshot | None = None
        self._target: float | None = None
        self._mask: tuple[tuple[bool, ...], ...] = ()
        self._stable_since: float | None = None
        self._stopped_at = -math.inf
        self._last_status: tuple[int, str] | None = None
        self._estimates: dict[tuple[int, int], tuple[float, float]] = {}
        self._previous_states: tuple[tuple[int, ...], ...] = ()

    def update(
        self,
        sample: TactileSnapshot,
        *,
        target_force_n: float,
        permitted: bool,
        now_s: float,
        worker: TactileWorker,
    ) -> list[dict[str, object]]:
        """按实测稳定性启动，并在证据充分、失接触或失鲜时请求停用。"""
        c = self.config
        events: list[dict[str, object]] = []
        status = worker.native_slip_status
        signature = (status.session_id, status.phase)
        if signature != self._last_status:
            events.append(
                {"event": "native_slip_state", **self.trace_fields(worker), "reason": status.reason}
            )
            if status.phase == "stopped":
                self._stopped_at = now_s
                self._stable_since = None
            self._last_status = signature
        if status.phase == "failed":
            raise RuntimeError(f"原厂滑移服务失败：{status.reason}")
        self._estimates = {
            key: value
            for key, value in self._estimates.items()
            if now_s - value[1] <= c.estimate_expiry_s
        }
        previous = self._previous
        fresh = 0 <= now_s - sample.received_at_s <= c.sample_timeout_s
        new = previous is None or sample.timestamp_us > previous.timestamp_us
        if not fresh or not permitted:
            worker.stop_native_slip("stale_sample" if not fresh else "phase_exit")
            self._stable_since = None
            if not fresh:
                self._estimates.clear()
            return events
        if not new:
            if self._target is not None and target_force_n != self._target:
                self._stable_since = None
                self._target = target_force_n
            return events
        gap = previous is not None and (
            sample.received_at_s - previous.received_at_s > c.sample_timeout_s
            or (sample.timestamp_us - previous.timestamp_us) * 1e-6 > c.sample_timeout_s
            or bool(sample.counter_gap)
        )
        forces = (sample.left_taxel_forces_n, sample.right_taxel_forces_n)
        mask = tuple(
            tuple(
                math.isfinite(row[2])
                and row[2]
                >= (
                    c.contact_off_n
                    if len(self._mask) == 2 and i < len(self._mask[side]) and self._mask[side][i]
                    else c.contact_on_n
                )
                for i, row in enumerate(side_forces)
            )
            for side, side_forces in enumerate(forces)
        )
        changed = bool(self._mask) and mask != self._mask
        valid_contact = all(sum(side) >= c.min_estimates_per_side for side in mask)
        self._mask = mask
        if gap or changed or not valid_contact:
            self._estimates.clear()
            worker.stop_native_slip("sample_gap" if gap else "contact_changed")
            self._stable_since = None
        running = status.phase in {"queued", "starting", "active", "stopping"}
        if running:
            self._stable_since = None
            # 原厂 SLIPPED 是状态而非每帧新事件；只有本会话的新转换产生证据。
            if (
                status.phase == "active"
                and sample.native_session_id == status.session_id
                and sample.native_session_phase == "active"
                and not (gap or changed)
                and valid_contact
                and sample.native_slip_active == (True, True)
            ):
                states, estimates = sample.native_pillar_states, sample.native_pillar_friction
                if len(states) == len(estimates) == 2:
                    for side in range(2):
                        if len(states[side]) != len(mask[side]) or len(estimates[side]) != len(
                            mask[side]
                        ):
                            continue
                        for i, state in enumerate(states[side]):
                            old_state = (
                                self._previous_states[side][i]
                                if len(self._previous_states) == 2
                                and i < len(self._previous_states[side])
                                else None
                            )
                            mu = estimates[side][i]
                            if (
                                mask[side][i]
                                and state == 3
                                and old_state != 3
                                and mu is not None
                                and math.isfinite(mu)
                                and mu > 0
                            ):
                                self._estimates[(side, i)] = (mu, now_s)
                                events.append(
                                    {
                                        "event": "native_slip_estimate",
                                        "session_id": self.session_id,
                                        "side": "left" if side == 0 else "right",
                                        "pillar_id": i,
                                        "native_mu": mu,
                                        "device_timestamp_us": sample.timestamp_us,
                                        "packet_counter": sample.packet_counter,
                                        "received_at_s": sample.received_at_s,
                                    }
                                )
                    if all(
                        sum(key[0] == side for key in self._estimates) >= c.min_estimates_per_side
                        for side in range(2)
                    ):
                        worker.stop_native_slip("estimates_ready")
            # 启动确认前收到的第一帧 SLIPPED 仍可在确认后消费一次。
            if (
                status.phase == "active"
                and sample.native_session_id == status.session_id
                and sample.native_session_phase == "active"
                and sample.native_slip_active == (True, True)
                and len(sample.native_pillar_states) == 2
                and all(
                    len(states) == len(side_mask)
                    for states, side_mask in zip(sample.native_pillar_states, mask, strict=True)
                )
            ):
                self._previous_states = sample.native_pillar_states
        elif (
            not gap
            and not changed
            and valid_contact
            and self.session_id < c.max_sessions
            and not self._estimates
            and now_s - self._stopped_at >= c.cooldown_s
        ):
            dt = 0.0 if previous is None else (sample.timestamp_us - previous.timestamp_us) * 1e-6
            normal = (sample.left_force_n, sample.right_force_n)
            stable = (
                previous is not None
                and dt > 0
                and all(
                    math.isfinite(value)
                    and abs(value - target_force_n) <= c.force_tolerance_n
                    and abs(value - old) / dt <= c.force_rate_n_s
                    for value, old in zip(
                        normal, (previous.left_force_n, previous.right_force_n), strict=True
                    )
                )
                and self._target is not None
                and abs(target_force_n - self._target) / dt <= c.force_rate_n_s
            )
            if stable:
                self._stable_since = (
                    sample.received_at_s if self._stable_since is None else self._stable_since
                )
                if sample.received_at_s - self._stable_since >= c.stable_duration_s:
                    self.session_id += 1
                    self._previous_states = sample.native_pillar_states
                    worker.request_native_slip(
                        self.session_id,
                        max_duration_s=c.max_duration_s,
                        confirmation_timeout_s=c.confirmation_timeout_s,
                    )
                    self._stable_since = None
            else:
                self._stable_since = None
        self._previous, self._target = sample, target_force_n
        return events

    def trace_fields(self, worker: TactileWorker) -> dict[str, object]:
        """记录租期状态和有效结果数量，逐点估计另存事件及原始日志。"""
        status = worker.native_slip_status
        return {
            "native_session_id": status.session_id,
            "native_session_phase": status.phase,
            "native_session_reason": status.reason,
            "native_left_estimate_count": sum(key[0] == 0 for key in self._estimates),
            "native_right_estimate_count": sum(key[0] == 1 for key in self._estimates),
        }
