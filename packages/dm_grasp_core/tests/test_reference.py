"""共享核目标力曲线的插值与导数契约测试。"""

from __future__ import annotations

import math

import pytest

from dm_grasp_core import ForceReferenceCurve, ForceWaypoint, sample_force_reference


def test_curve_rejects_invalid_waypoint_sequences():
    """单点、时间倒序、非有限与负值 waypoint 必须被拒绝。"""
    with pytest.raises(ValueError, match="at least two"):
        ForceReferenceCurve(waypoints=(ForceWaypoint(0.0, 0.5),))
    with pytest.raises(ValueError, match="strictly increasing"):
        ForceReferenceCurve(
            waypoints=(ForceWaypoint(0.0, 0.5), ForceWaypoint(1.0, 0.6), ForceWaypoint(1.0, 0.7))
        )
    with pytest.raises(ValueError, match="finite"):
        ForceWaypoint(t_s=math.inf, force_n=0.5)
    with pytest.raises(ValueError, match="non-negative"):
        ForceWaypoint(t_s=-0.1, force_n=0.5)
    with pytest.raises(ValueError, match="non-negative"):
        ForceWaypoint(t_s=0.0, force_n=-0.1)
    with pytest.raises(ValueError, match="unknown"):
        ForceReferenceCurve(
            interpolation="cubic", waypoints=(ForceWaypoint(0, 1), ForceWaypoint(1, 1))
        )


def test_curve_hold_uses_left_value_until_exact_boundary():
    """hold 在左闭右开区间取左值，越过边界后保持右值，导数为零。"""
    curve = ForceReferenceCurve(
        interpolation="hold",
        waypoints=(ForceWaypoint(0.0, 0.4), ForceWaypoint(2.0, 0.8)),
    )
    assert curve.target_at(0.0) == pytest.approx(0.4)
    assert curve.target_at(1.999999) == pytest.approx(0.4)
    assert curve.target_at(2.0) == pytest.approx(0.4)
    assert curve.target_at(2.000001) == pytest.approx(0.8)
    force, rate, acceleration = curve.sample_at(1.0)
    assert force == pytest.approx(0.4)
    assert rate == 0.0
    assert acceleration == 0.0


def test_curve_linear_derivatives_are_constant():
    """linear 区间导数为常数，末点之后保持末值。"""
    curve = ForceReferenceCurve(
        interpolation="linear",
        waypoints=(ForceWaypoint(0.0, 0.5), ForceWaypoint(2.0, 1.0)),
    )
    force, rate, acceleration = curve.sample_at(1.0)
    assert force == pytest.approx(0.75)
    assert rate == pytest.approx(0.25)
    assert acceleration == 0.0
    assert curve.target_at(-1.0) == pytest.approx(0.5)
    assert curve.target_at(5.0) == pytest.approx(1.0)


def test_curve_smoothstep_matches_reference_semantics():
    """smoothstep 中点取半、端点导数为零、二阶导对称。"""
    curve = ForceReferenceCurve(
        interpolation="smoothstep",
        waypoints=(ForceWaypoint(0.0, 0.4), ForceWaypoint(2.0, 0.8)),
    )
    mid_force, mid_rate, mid_acceleration = curve.sample_at(1.0)
    assert mid_force == pytest.approx(0.6)
    assert mid_rate == pytest.approx(0.4 / 2.0 * 1.5)
    assert mid_acceleration == pytest.approx(0.0)
    _, start_rate, _ = curve.sample_at(1e-9)
    assert start_rate == pytest.approx(0.0, abs=1e-6)
    assert curve.duration_s == pytest.approx(2.0)


def test_sample_function_accepts_duck_typed_waypoints():
    """纯函数接受任何带 t_s／force_n 属性的 waypoint 序列。"""

    class LegacyWaypoint:
        def __init__(self, t_s, force_n):
            self.t_s = t_s
            self.force_n = force_n

    force, rate, _ = sample_force_reference(
        "linear",
        [LegacyWaypoint(0.0, 0.5), LegacyWaypoint(2.0, 1.5)],
        1.0,
    )
    assert force == pytest.approx(1.0)
    assert rate == pytest.approx(0.5)
