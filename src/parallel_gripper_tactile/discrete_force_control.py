"""基于单 tick 力增量的 Robotiq 离散力控制器。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import StrEnum
import math
from typing import Literal

import numpy as np


DiscreteControllerVariant = Literal[
    "fixed-step",
    "adaptive-deadband",
    "predictive",
    "dynamic-step",
]


class DiscreteControlState(StrEnum):
    """离散控制状态。"""

    APPROACH = "APPROACH"
    WAIT_STABLE = "WAIT_STABLE"
    ADJUST = "ADJUST"
    HOLD = "HOLD"
    RELEASE = "RELEASE"


@dataclass(frozen=True, slots=True)
class DiscreteForceControlConfig:
    """离散状态机所需的全部控制参数。"""

    command_min: int = 0
    command_max: int = 255
    approach_step: int = 3
    approach_action_interval_s: float = 0.05
    max_dynamic_step: int = 3
    target_force_n: float = 5.0
    max_force_n: float = 8.0
    contact_threshold_n: float = 0.2
    stable_window_samples: int = 20
    stable_force_range_n: float = 0.1
    min_settle_time_s: float = 0.2
    delta_f_ema_alpha: float = 0.3
    delta_f_min_valid_samples: int = 3
    delta_f_minimum_n: float = 0.02
    fixed_deadband_n: float = 0.15
    fixed_reactivate_margin_n: float = 0.1
    noise_sigma_factor: float = 3.0
    tick_deadband_factor: float = 0.6
    reactivate_tick_factor: float = 0.7
    reactivate_duration_s: float = 0.2
    prediction_noise_factor: float = 2.0
    prediction_tick_margin_factor: float = 0.2
    dynamic_step_eta: float = 0.6
    safety_force_margin_n: float = 0.3

    def __post_init__(self) -> None:
        """拒绝会破坏整数命令、阈值滞回或安全边界的配置。"""
        if self.command_min < 0 or self.command_min >= self.command_max:
            raise ValueError("command range must be non-negative and increasing")
        if self.approach_step < 1 or self.max_dynamic_step < 1:
            raise ValueError("action steps must be positive integers")
        if self.target_force_n <= 0 or self.max_force_n <= self.target_force_n:
            raise ValueError("max_force_n must exceed the positive target force")
        if self.contact_threshold_n <= 0 or self.contact_threshold_n >= self.target_force_n:
            raise ValueError("contact threshold must lie between zero and target force")
        if self.stable_window_samples < 2:
            raise ValueError("stable_window_samples must be at least two")
        positive = (
            self.stable_force_range_n,
            self.min_settle_time_s,
            self.approach_action_interval_s,
            self.delta_f_minimum_n,
            self.fixed_deadband_n,
            self.reactivate_duration_s,
            self.dynamic_step_eta,
            self.safety_force_margin_n,
        )
        if any(not math.isfinite(value) or value <= 0 for value in positive):
            raise ValueError(
                "force, timing, and dynamic-step parameters must be finite and positive"
            )
        if self.delta_f_min_valid_samples < 1:
            raise ValueError("delta_f_min_valid_samples must be positive")
        if not 0 < self.delta_f_ema_alpha <= 1:
            raise ValueError("delta_f_ema_alpha must lie in (0, 1]")
        nonnegative = (
            self.fixed_reactivate_margin_n,
            self.noise_sigma_factor,
            self.tick_deadband_factor,
            self.reactivate_tick_factor,
            self.prediction_noise_factor,
            self.prediction_tick_margin_factor,
        )
        if any(not math.isfinite(value) or value < 0 for value in nonnegative):
            raise ValueError("threshold factors must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class DiscreteControlSnapshot:
    """某个采样时刻可记录的控制器内部状态。"""

    state: DiscreteControlState
    command: int
    requested_delta: int
    force_mean_n: float
    force_noise_std_n: float
    force_range_n: float
    force_stable: bool
    delta_f_tick_raw_n: float | None
    delta_f_tick_estimate_n: float | None
    delta_f_tick_samples: int
    hold_deadband_n: float
    reactivate_threshold_n: float
    prediction_margin_n: float
    predicted_force_next_n: float | None
    predicted_error_next_n: float | None
    dynamic_step_raw: float | None
    dynamic_step_command: int
    action_count: int
    total_command_movement: int
    reverse_count: int


class DiscreteForceController:
    """在整数 tendon 命令空间中执行稳定后再动作的力控制。"""

    def __init__(
        self,
        config: DiscreteForceControlConfig,
        *,
        variant: DiscreteControllerVariant = "dynamic-step",
    ) -> None:
        """初始化控制状态、稳定窗口和局部 tick 增益估计。"""
        if variant not in {
            "fixed-step",
            "adaptive-deadband",
            "predictive",
            "dynamic-step",
        }:
            raise ValueError(f"unsupported discrete controller variant: {variant}")
        self.config = config
        self.variant = variant
        self._target_force_n = config.target_force_n
        self.state = DiscreteControlState.APPROACH
        self.command = config.command_min
        self._samples: deque[float] = deque(maxlen=config.stable_window_samples)
        self._last_sample = 0.0
        self._last_action_time_s = -math.inf
        self._pending_action = False
        self._wait_reason = "contact"
        self._force_before_action_n: float | None = None
        self._action_for_estimate = 0
        self._reactivate_since_s: float | None = None
        self._delta_f_tick_estimate_n: float | None = None
        self._delta_f_tick_raw_n: float | None = None
        self._delta_f_tick_samples = 0
        self._last_nonzero_delta = 0
        self.action_count = 0
        self.total_command_movement = 0
        self.reverse_count = 0
        self._requested_delta = 0
        self._predicted_force_next_n: float | None = None
        self._predicted_error_next_n: float | None = None
        self._pending_predicted_force_n: float | None = None
        self._active_predicted_force_n: float | None = None
        self.prediction_errors_n: list[float] = []
        self._dynamic_step_raw: float | None = None
        self._dynamic_step_command = 0

    @property
    def delta_f_tick_reliable(self) -> bool:
        """返回局部 tick 增益是否已积累足够有效样本。"""
        return (
            self._delta_f_tick_estimate_n is not None
            and self._delta_f_tick_samples >= self.config.delta_f_min_valid_samples
        )

    @property
    def target_force_n(self) -> float:
        """返回当前时变目标力。"""
        return self._target_force_n

    def set_target_force(self, target_force_n: float) -> None:
        """更新目标力，同时保持已经辨识的局部 tick 增益。"""
        value = float(target_force_n)
        if not math.isfinite(value) or value <= 0 or value >= self.config.max_force_n:
            raise ValueError("target force must be finite and lie below max_force_n")
        self._target_force_n = value

    def observe(self, force_n: float) -> None:
        """把一个有限的滤波力采样加入稳定性窗口。"""
        value = float(force_n)
        if not math.isfinite(value):
            raise ValueError("force sample must be finite")
        self._last_sample = value
        self._samples.append(value)

    def _window_statistics(self) -> tuple[float, float, float, bool]:
        values = np.asarray(self._samples, dtype=np.float64)
        if values.size == 0:
            return self._last_sample, 0.0, math.inf, False
        mean = float(np.mean(values))
        noise = float(np.std(values))
        force_range = float(np.ptp(values))
        stable = (
            values.size == self.config.stable_window_samples
            and force_range < self.config.stable_force_range_n
        )
        return mean, noise, force_range, stable

    def _thresholds(self) -> tuple[float, float, float]:
        _, noise, _, _ = self._window_statistics()
        tick = self._delta_f_tick_estimate_n if self.delta_f_tick_reliable else None
        adaptive = self.variant != "fixed-step" and tick is not None
        hold = self.config.fixed_deadband_n
        if adaptive:
            hold = max(
                self.config.noise_sigma_factor * noise,
                self.config.tick_deadband_factor * tick,
            )
        reactivate = hold + self.config.fixed_reactivate_margin_n
        prediction = self.config.prediction_noise_factor * noise
        if adaptive:
            reactivate = hold + self.config.reactivate_tick_factor * tick
            prediction = max(
                prediction,
                self.config.prediction_tick_margin_factor * tick,
            )
        return hold, reactivate, prediction

    def _request(self, delta: int) -> int:
        requested = int(delta)
        bounded_command = int(
            np.clip(self.command + requested, self.config.command_min, self.config.command_max)
        )
        requested = bounded_command - self.command
        self._requested_delta = requested
        self._dynamic_step_command = requested
        if requested != 0:
            self._pending_action = True
            self._pending_predicted_force_n = self._predicted_force_next_n
        return requested

    def _prediction_improves(self, force_n: float, delta: int, margin_n: float) -> bool:
        tick = self._delta_f_tick_estimate_n
        if tick is None:
            return True
        predicted = force_n + delta * tick
        current_error = abs(self._target_force_n - force_n)
        predicted_error = abs(self._target_force_n - predicted)
        self._predicted_force_next_n = predicted
        self._predicted_error_next_n = predicted_error
        return predicted_error + margin_n < current_error

    def _tightening_step(self, force_n: float) -> int:
        tick = self._delta_f_tick_estimate_n
        if not self.delta_f_tick_reliable or tick is None:
            self._dynamic_step_raw = 1.0
            return 1
        error = self._target_force_n - force_n
        requested = 1
        if self.variant == "dynamic-step":
            raw = self.config.dynamic_step_eta * abs(error) / max(tick, 1e-12)
            self._dynamic_step_raw = raw
            requested = int(np.clip(math.floor(raw), 1, self.config.max_dynamic_step))
        else:
            self._dynamic_step_raw = 1.0
        safe = math.floor(
            (self.config.max_force_n - self.config.safety_force_margin_n - force_n)
            / max(tick, 1e-12)
        )
        return min(requested, max(0, safe))

    def decide(self, time_s: float) -> int:
        """根据当前状态返回本周期请求的整数命令增量。"""
        now = float(time_s)
        force, _, _, stable = self._window_statistics()
        hold, reactivate, prediction_margin = self._thresholds()
        self._requested_delta = 0
        self._dynamic_step_raw = None
        self._dynamic_step_command = 0
        self._predicted_force_next_n = None
        self._predicted_error_next_n = None
        if self._pending_action:
            return 0

        if self.state == DiscreteControlState.APPROACH:
            if force < self.config.contact_threshold_n:
                if now - self._last_action_time_s < self.config.approach_action_interval_s:
                    return 0
                return self._request(self.config.approach_step)
            self.state = DiscreteControlState.WAIT_STABLE
            self._wait_reason = "contact"
            return 0

        if self.state == DiscreteControlState.WAIT_STABLE:
            settled = now - self._last_action_time_s >= self.config.min_settle_time_s
            if stable and settled:
                self._finish_settled_action(force)
            return 0

        if force > self.config.max_force_n:
            self.state = DiscreteControlState.RELEASE

        if self.state == DiscreteControlState.RELEASE:
            if force <= self.config.max_force_n - self.config.safety_force_margin_n:
                self.state = DiscreteControlState.HOLD
                return 0
            return self._request(-1)

        error = self._target_force_n - force
        if self.state == DiscreteControlState.HOLD:
            if abs(error) > reactivate:
                if self._reactivate_since_s is None:
                    self._reactivate_since_s = now
                elif now - self._reactivate_since_s >= self.config.reactivate_duration_s:
                    self.state = DiscreteControlState.ADJUST
                    self._reactivate_since_s = None
            else:
                self._reactivate_since_s = None
            return 0

        if abs(error) <= hold:
            self.state = DiscreteControlState.HOLD
            return 0

        if error > 0:
            step = self._tightening_step(force)
            if step <= 0:
                self.state = DiscreteControlState.HOLD
                return 0
            if self.variant in {"predictive", "dynamic-step"} and not self._prediction_improves(
                force, step, prediction_margin
            ):
                self.state = DiscreteControlState.HOLD
                return 0
            return self._request(step)

        # 松开始终保持单 tick，并使用比夹紧更保守的一步预测。
        if self.delta_f_tick_reliable and not self._prediction_improves(
            force, -1, prediction_margin
        ):
            self.state = DiscreteControlState.HOLD
            return 0
        return self._request(-1)

    def action_applied(self, delta: int, time_s: float) -> None:
        """在延迟队列真正修改 actuator 命令时登记一次动作。"""
        actual = int(delta)
        if actual == 0:
            self._pending_action = False
            return
        if (
            self.command + actual < self.config.command_min
            or self.command + actual > self.config.command_max
        ):
            raise ValueError("applied action would exceed command limits")
        force, _, _, stable = self._window_statistics()
        self.command += actual
        self._pending_action = False
        self._active_predicted_force_n = self._pending_predicted_force_n
        self._pending_predicted_force_n = None
        self._last_action_time_s = float(time_s)
        self.action_count += 1
        self.total_command_movement += abs(actual)
        if self._last_nonzero_delta * actual < 0:
            self.reverse_count += 1
        self._last_nonzero_delta = actual
        if self.state == DiscreteControlState.APPROACH:
            return
        self._force_before_action_n = force if stable else self._last_sample
        self._action_for_estimate = actual
        self._wait_reason = "release" if self.state == DiscreteControlState.RELEASE else "adjust"
        self.state = DiscreteControlState.WAIT_STABLE

    def _finish_settled_action(self, force_n: float) -> None:
        if self._active_predicted_force_n is not None:
            self.prediction_errors_n.append(abs(force_n - self._active_predicted_force_n))
        self._active_predicted_force_n = None
        if (
            self._wait_reason == "adjust"
            and self._action_for_estimate > 0
            and self._force_before_action_n is not None
            and self._force_before_action_n >= self.config.contact_threshold_n
        ):
            raw = (force_n - self._force_before_action_n) / self._action_for_estimate
            if math.isfinite(raw) and raw >= self.config.delta_f_minimum_n:
                self._delta_f_tick_raw_n = raw
                if self._delta_f_tick_estimate_n is None:
                    self._delta_f_tick_estimate_n = raw
                else:
                    alpha = self.config.delta_f_ema_alpha
                    self._delta_f_tick_estimate_n = (
                        1 - alpha
                    ) * self._delta_f_tick_estimate_n + alpha * raw
                self._delta_f_tick_samples += 1
        self._force_before_action_n = None
        self._action_for_estimate = 0
        if self._wait_reason == "release" and force_n > (
            self.config.max_force_n - self.config.safety_force_margin_n
        ):
            self.state = DiscreteControlState.RELEASE
        else:
            self.state = DiscreteControlState.ADJUST

    def snapshot(self) -> DiscreteControlSnapshot:
        """返回不暴露内部可变窗口的诊断快照。"""
        mean, noise, force_range, stable = self._window_statistics()
        hold, reactivate, prediction = self._thresholds()
        return DiscreteControlSnapshot(
            state=self.state,
            command=self.command,
            requested_delta=self._requested_delta,
            force_mean_n=mean,
            force_noise_std_n=noise,
            force_range_n=force_range,
            force_stable=stable,
            delta_f_tick_raw_n=self._delta_f_tick_raw_n,
            delta_f_tick_estimate_n=self._delta_f_tick_estimate_n,
            delta_f_tick_samples=self._delta_f_tick_samples,
            hold_deadband_n=hold,
            reactivate_threshold_n=reactivate,
            prediction_margin_n=prediction,
            predicted_force_next_n=self._predicted_force_next_n,
            predicted_error_next_n=self._predicted_error_next_n,
            dynamic_step_raw=self._dynamic_step_raw,
            dynamic_step_command=self._dynamic_step_command,
            action_count=self.action_count,
            total_command_movement=self.total_command_movement,
            reverse_count=self.reverse_count,
        )


__all__ = [
    "DiscreteControllerVariant",
    "DiscreteControlSnapshot",
    "DiscreteControlState",
    "DiscreteForceControlConfig",
    "DiscreteForceController",
]
