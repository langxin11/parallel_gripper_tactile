"""零力窗口验证的行为锚点：设备常数下的空载门禁与告警。"""

from __future__ import annotations

from dataclasses import replace

import pytest

from .fakes import FakeClock, FakeTactile, PhaseActions, make_config

from dmgripper_experiments.config import ExperimentConfig
from dmgripper_experiments.runtime import (
    _TACTILE_BIAS_SETTLE_S,
    _ZERO_FORCE_STABLE_S,
    _ZERO_FORCE_THRESHOLD_N,
    _verify_zero,
)
from dmgripper_experiments.tactile import TactileSnapshot


def _zero_config() -> ExperimentConfig:
    """返回零力验证用的统一自适应配置。"""
    return make_config()


def test_verify_zero_uses_filtered_fz_and_tolerates_raw_noise() -> None:
    """零力门禁使用滤波 Fz，原始三轴噪声尖峰不应阻断验证。"""
    clock = FakeClock()
    actions = PhaseActions()

    class NoisyTactile(FakeTactile):
        """持续返回滤波力为零、原始三轴合力非零的快照。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            """推进时钟并构造含原始噪声的触觉帧。"""
            self.clock.advance(0.01)
            snapshot = self._snapshot(force_n=0.0)
            return replace(snapshot, raw_left_fz_n=0.2, raw_right_fz_n=0.2)

    tactile = NoisyTactile(None, clock=clock, phase=actions)
    _verify_zero(tactile, _zero_config(), False, clock, clock.sleep)


def test_verify_zero_waits_for_a_new_packet_after_bias_settle() -> None:
    """settle 完成时重新划定边界，验证窗口不得复用 settle 期间的旧包。"""
    clock = FakeClock()
    actions = PhaseActions()

    class BoundaryTactile(FakeTactile):
        def __init__(self, *args, **kwargs) -> None:
            super().__init__(*args, **kwargs)
            self.previous_boundaries: list[float | None] = []

        def wait_for_update(self, previous_received_at_s, _timeout_s) -> TactileSnapshot:
            self.previous_boundaries.append(previous_received_at_s)
            self.clock.advance(0.01)
            return self._snapshot(force_n=0.0)

    tactile = BoundaryTactile(None, clock=clock, phase=actions)
    _verify_zero(tactile, _zero_config(), True, clock, clock.sleep)
    assert tactile.previous_boundaries[0] is None
    assert tactile.previous_boundaries[1] == pytest.approx(0.01 + _TACTILE_BIAS_SETTLE_S)


def test_verify_zero_accepts_window_mean_below_threshold() -> None:
    """滤波噪声在窗口均值阈值以下时应通过验证。"""
    clock = FakeClock()
    actions = PhaseActions()

    class MeanStableTactile(FakeTactile):
        """交替返回位于均值阈值两侧的空载噪声。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            """推进时钟并构造均值低于阈值的滤波噪声。"""
            self.clock.advance(0.01)
            force_n = 0.11 if self.counter % 2 == 0 else 0.08
            return self._snapshot(force_n=force_n)

    tactile = MeanStableTactile(None, clock=clock, phase=actions)
    _verify_zero(tactile, _zero_config(), False, clock, clock.sleep)


def test_verify_zero_contact_peak_emits_warning_without_blocking() -> None:
    """接触级峰值只告警，合格的窗口均值仍通过零力验证。"""
    clock = FakeClock()
    actions = PhaseActions()
    warnings: list[dict[str, object]] = []
    threshold = _zero_config().lifecycle.contact_on_n

    class ContactPeakTactile(FakeTactile):
        """每三个采样注入一次达到接触阈值的双侧峰值。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            """推进时钟并构造周期性接触峰值。"""
            self.clock.advance(0.01)
            force_n = threshold if self.counter % 3 == 0 else 0.0
            return self._snapshot(force_n=force_n)

    tactile = ContactPeakTactile(None, clock=clock, phase=actions)
    _verify_zero(
        tactile,
        _zero_config(),
        False,
        clock,
        clock.sleep,
        warning_sink=warnings.append,
    )
    assert warnings[0]["event"] == "zero_force_diagnostics"
    peak_warning = next(warning for warning in warnings[1:] if warning.get("event") == "warning")
    assert peak_warning["code"] == "zero_force_peak"
    assert peak_warning["peak_side"] == "both"
    assert peak_warning["maximum_peak_n"] == pytest.approx(threshold)
    assert peak_warning["peak_warning_threshold_n"] == pytest.approx(threshold)
    diagnostics = warnings[0]
    taxels = diagnostics["taxels"]
    assert taxels["left"]["taxel_count"] == 9
    assert taxels["left"]["maximum_abs_residual_n"] == pytest.approx(threshold / 3)


def test_verify_zero_rejects_contact_peak_when_window_mean_is_high() -> None:
    """峰值告警不能绕过双侧窗口均值门禁。"""
    clock = FakeClock()
    actions = PhaseActions()
    warnings: list[dict[str, object]] = []

    class SustainedPeakTactile(FakeTactile):
        """持续返回达到接触阈值的滤波力。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            self.clock.advance(0.01)
            return self._snapshot(force_n=0.2)

    tactile = SustainedPeakTactile(None, clock=clock, phase=actions)
    with pytest.raises(RuntimeError, match=r"均值 LEFT=0\.200N"):
        _verify_zero(
            tactile,
            _zero_config(),
            False,
            clock,
            clock.sleep,
            warning_sink=warnings.append,
        )
    assert warnings == []


def test_verify_zero_reports_filtered_force_timeout_instead_of_snapshot_timeout() -> None:
    """持续滤波残余力应报告零力统计，期限末端不能误报触觉断流。"""
    clock = FakeClock()
    actions = PhaseActions()

    class LoadedTactile(FakeTactile):
        """持续返回略高于零力阈值的滤波法向力。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            """推进时钟并构造持续受载触觉帧。"""
            self.clock.advance(0.01)
            return self._snapshot(force_n=0.11)

    tactile = LoadedTactile(None, clock=clock, phase=actions)
    with pytest.raises(RuntimeError, match=r"滤波双侧 Fz.*均值 LEFT=0\.110N"):
        _verify_zero(tactile, _zero_config(), False, clock, clock.sleep)


def test_verify_zero_window_constants_are_device_behavior() -> None:
    """零力窗口参数已固化为运行时常数，不再来自配置。"""
    assert _ZERO_FORCE_THRESHOLD_N == pytest.approx(0.1)
    assert _ZERO_FORCE_STABLE_S == pytest.approx(0.5)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf")])
def test_verify_zero_rejects_nonfinite_taxel(bad_value: float) -> None:
    """任一逐 taxel 分量非有限时必须阻断使能前验证。"""
    clock = FakeClock()
    actions = PhaseActions()

    class BadTaxelTactile(FakeTactile):
        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            self.clock.advance(0.01)
            return replace(
                self._snapshot(force_n=0.0),
                left_taxel_forces_n=((bad_value, 0.0, 0.0),) * 9,
            )

    tactile = BadTaxelTactile(None, clock=clock, phase=actions)
    with pytest.raises(RuntimeError, match="taxel 0.*非有限"):
        _verify_zero(tactile, _zero_config(), False, clock, clock.sleep)


def test_verify_zero_rejects_taxel_count_change() -> None:
    """同次零力窗口内任一侧 taxel 数变化必须阻断启动。"""
    clock = FakeClock()
    actions = PhaseActions()

    class ChangingTaxelTactile(FakeTactile):
        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            self.clock.advance(0.01)
            snapshot = self._snapshot(force_n=0.0)
            if self.counter >= 2:
                return replace(
                    snapshot,
                    left_taxel_forces_n=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                )
            return snapshot

    tactile = ChangingTaxelTactile(None, clock=clock, phase=actions)
    # 九触点数量约束属统一策略保护；这里用未启用统一的配置隔离窗口行为。
    with pytest.raises(RuntimeError, match="taxel 数量变化"):
        _verify_zero(tactile, ExperimentConfig(), False, clock, clock.sleep)


def test_local_taxel_residual_is_diagnostic_not_zero_gate() -> None:
    """局部残余只进入诊断事件，全局窗口均值仍决定通过。"""
    clock = FakeClock()
    actions = PhaseActions()
    events: list[dict[str, object]] = []

    class ResidualTaxelTactile(FakeTactile):
        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            self.clock.advance(0.01)
            return replace(
                self._snapshot(force_n=0.0),
                left_taxel_forces_n=((0.0, 0.0, 5.0),) * 9,
            )

    tactile = ResidualTaxelTactile(None, clock=clock, phase=actions)
    _verify_zero(
        tactile,
        _zero_config(),
        False,
        clock,
        clock.sleep,
        warning_sink=events.append,
    )
    diagnostic = next(event for event in events if event["event"] == "zero_force_diagnostics")
    assert diagnostic["taxels"]["left"]["maximum_abs_residual_n"] == pytest.approx(5.0)
