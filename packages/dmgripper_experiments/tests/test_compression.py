"""验证基于非线性关节运动学的接触压缩保护。"""

import math

import pytest

from dm_grasp_core import CrankSliderKinematics
from dmgripper_experiments.compression import ContactCompressionGuard


@pytest.fixture
def kinematics() -> CrankSliderKinematics:
    """提供实际曲柄滑块参数。"""
    return CrankSliderKinematics(math.pi / 4, 0.03, 0.04, 0.021213203435596423)


@pytest.mark.parametrize("forces", [(0.5, 0.0), (0.0, 0.5)])
def test_first_contact_and_persistent_baseline(kinematics, forces) -> None:
    """任一侧到阈值即锁定，失去接触仍保留累积压缩。"""
    guard = ContactCompressionGuard(kinematics, 0.02, 0.5)
    guard.observe(0.2, 0.1, -0.1)
    assert guard.compression_m is None
    guard.observe(0.3, *forces)
    assert guard.compression_m == 0.0
    guard.observe(0.5, 0.0, 0.0)
    assert guard.compression_m == pytest.approx(kinematics.closure(0.5) - kinematics.closure(0.3))
    assert guard.trace_fields()["contact_closure_m"] == kinematics.closure(0.3)
    guard.observe(0.2, 0.8, 0.8)
    assert guard.compression_m == 0.0
    guard.observe(0.55, 0.8, 0.8)
    assert guard.compression_m == pytest.approx(kinematics.closure(0.55) - kinematics.closure(0.3))
    guard.reset()
    assert guard.trace_fields() == {
        "contact_closure_m": None,
        "contact_compression_m": None,
        "contact_compression_limit_m": 0.02,
    }


def test_no_contact_does_not_limit_free_travel(kinematics) -> None:
    """无接触基线时允许超过压缩限值的空载闭合行程。"""
    guard = ContactCompressionGuard(kinematics, 0.001, 0.5)
    guard.observe(0.8, 0.1, 0.1)
    guard.check_command(1.0, 0.2, 0.01)
    assert guard.compression_m is None


@pytest.mark.parametrize("position", [0.7, 0.8])
def test_observation_rejects_at_or_above_limit(kinematics, position) -> None:
    """测量到达边界或超限均中止，保留触发时压缩记录。"""
    limit = kinematics.closure(0.7) - kinematics.closure(0.3)
    guard = ContactCompressionGuard(kinematics, limit, 0.5)
    guard.observe(0.3, 0.5, 0.5)
    with pytest.raises(RuntimeError, match="最大压缩"):
        guard.observe(position, 0.5, 0.5)
    assert guard.compression_m >= limit


def test_command_checks_target_and_velocity_from_measured_position(kinematics) -> None:
    """位置目标及速度预测独立受限，并允许向开口方向撤离。"""
    limit = kinematics.closure(0.7) - kinematics.closure(0.3)
    guard = ContactCompressionGuard(kinematics, limit, 0.5)
    guard.observe(0.3, 0.5, 0.5)
    guard.check_command(0.7, 0.0, 0.1)
    with pytest.raises(RuntimeError, match="最大压缩"):
        guard.check_command(0.71, -0.1, 0.1)
    guard.observe(0.6, 0.5, 0.5)
    with pytest.raises(RuntimeError, match="最大压缩"):
        guard.check_command(0.4, 1.1, 0.1)
    guard.check_command(0.4, -1.0, 0.1)


def test_disabled_limit_still_tracks_contact(kinematics) -> None:
    """关闭保护仍计算接触压缩并校验输入。"""
    guard = ContactCompressionGuard(kinematics, None, 0.5)
    guard.observe(0.1, 0.5, 0.0)
    guard.observe(0.8, 0.5, 0.5)
    guard.check_command(1.0, 1.0, 0.1)
    assert guard.compression_m == pytest.approx(kinematics.closure(0.8) - kinematics.closure(0.1))
    assert guard.trace_fields()["contact_compression_limit_m"] is None


@pytest.mark.parametrize("bad", [0.0, -0.1, math.nan, math.inf, True, "0.02"])
@pytest.mark.parametrize("parameter", ["max_compression_m", "contact_on_n"])
def test_invalid_parameters(kinematics, bad, parameter) -> None:
    """正参数拒绝零、负值、非有限值及非数值。"""
    kwargs = {"max_compression_m": 0.02, "contact_on_n": 0.5}
    kwargs[parameter] = bad
    with pytest.raises(ValueError):
        ContactCompressionGuard(kinematics, **kwargs)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, True, "0.2"])
@pytest.mark.parametrize("index", [0, 1, 2])
def test_observation_rejects_invalid_inputs(kinematics, bad, index) -> None:
    """未接触及关闭保护均不能掩盖非法反馈。"""
    guard = ContactCompressionGuard(kinematics, None, 0.5)
    values = [0.3, 0.5, 0.5]
    values[index] = bad
    with pytest.raises(ValueError):
        guard.observe(*values)
    assert guard.compression_m is None


@pytest.mark.parametrize(
    "index,bad", [(0, math.nan), (1, math.inf), (2, 0), (2, -0.1), (2, math.nan)]
)
def test_command_rejects_invalid_inputs(kinematics, index, bad) -> None:
    """无接触时命令仍必须有限且时间步长为正。"""
    guard = ContactCompressionGuard(kinematics, None, 0.5)
    values = [0.3, 0.0, 0.01]
    values[index] = bad
    with pytest.raises(ValueError):
        guard.check_command(*values)
