"""噪声感知零力窗口验证的行为锚点（自根目录 cup 测试迁移）。"""

from __future__ import annotations

from dataclasses import replace

import pytest

from .fakes import FakeClock, FakeTactile, PhaseActions, curve_config

from dmgripper_experiments.runtime import _verify_zero
from dmgripper_experiments.tactile import TactileSnapshot


def _zero_config(**lifecycle_overrides):
    """返回零力验证用的短时限配置。"""
    config = curve_config()
    lifecycle = replace(config.lifecycle, **lifecycle_overrides)
    return replace(config, lifecycle=lifecycle)


def test_verify_zero_uses_filtered_fz_and_tolerates_raw_noise() -> None:
    """零力门禁与 ROS 一致使用滤波 Fz，原始三轴噪声不应反复清空窗口。"""
    clock = FakeClock()
    actions = PhaseActions()

    class NoisyTactile(FakeTactile):
        """持续返回滤波力为零、原始三轴合力超过零力阈值的快照。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            """推进时钟并构造含原始噪声尖峰的触觉帧。"""
            self.clock.advance(0.01)
            snapshot = self._snapshot(force_n=0.0)
            return replace(snapshot, raw_left_fz_n=0.2, raw_right_fz_n=0.2)

    tactile = NoisyTactile(None, clock=clock, phase=actions)
    _verify_zero(tactile, _zero_config(), False, clock, clock.sleep)


def test_verify_zero_accepts_noise_crossing_mean_threshold() -> None:
    """滤波噪声可越过均值阈值，但窗口均值合格且未达接触峰值时应通过。"""
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
    _verify_zero(
        tactile,
        _zero_config(zero_force_stable_s=0.04),
        False,
        clock,
        clock.sleep,
    )


def test_verify_zero_contact_peak_emits_warning_without_resetting_window() -> None:
    """接触级峰值只告警，合格的窗口均值仍可通过零力验证。"""
    clock = FakeClock()
    actions = PhaseActions()
    warnings: list[dict[str, object]] = []

    class ContactPeakTactile(FakeTactile):
        """每三个采样注入一次达到接触阈值的双侧峰值。"""

        def wait_for_update(self, _previous_received_at_s, _timeout_s) -> TactileSnapshot:
            """推进时钟并构造周期性接触峰值。"""
            self.clock.advance(0.01)
            force_n = 0.2 if self.counter % 3 == 0 else 0.0
            return self._snapshot(force_n=force_n)

    tactile = ContactPeakTactile(None, clock=clock, phase=actions)
    _verify_zero(
        tactile,
        _zero_config(zero_force_stable_s=0.04, zero_force_timeout_s=0.12),
        False,
        clock,
        clock.sleep,
        warning_sink=warnings.append,
    )
    assert warnings == [
        {
            "event": "warning",
            "code": "zero_force_peak",
            "message": (
                "使能前零力均值验证通过，但滤波双侧 Fz 峰值为 0.200N（双侧），达到 "
                "0.200N 警告阈值；请确认传感器无持续受力"
            ),
            "maximum_peak_n": pytest.approx(0.2),
            "peak_side": "both",
            "peak_warning_threshold_n": pytest.approx(0.2),
        }
    ]


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
            _zero_config(zero_force_stable_s=0.04, zero_force_timeout_s=0.12),
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
