"""验证刚度归一化力变化率控制器。"""

from __future__ import annotations

import pytest

from dm_grasp_core import StiffnessRateConfig, StiffnessRateController


def test_rate_controller_integrates_by_elapsed_time() -> None:
    """相同物理时长在不同控制周期下产生相同位置修正。"""
    config = StiffnessRateConfig(
        kp_s_inv=1.0,
        max_force_rate_n_s=10.0,
        max_joint_velocity_rad_s=1.0,
    )
    results = []
    for dt_s in (0.002, 0.004, 0.008):
        controller = StiffnessRateController(config, max_position_adjustment_rad=1.0)
        step = None
        for _ in range(round(1.0 / dt_s)):
            step = controller.step(
                force_error_n=1.0,
                measured_force_n=0.0,
                stiffness_n_per_m=1000.0,
                closure_jacobian_m_per_rad=0.1,
                dt_s=dt_s,
            )
        assert step is not None
        results.append(step.position_adjustment_rad)
    assert results == pytest.approx([0.01, 0.01, 0.01])


def test_rate_controller_maps_force_rate_and_freezes_integral_at_limit() -> None:
    """力变化率和关节速度均受限，持续同向误差不产生积分 windup。"""
    controller = StiffnessRateController(
        StiffnessRateConfig(
            kp_s_inv=1.0,
            ki_s_inv2=2.0,
            max_force_rate_n_s=0.5,
            max_joint_velocity_rad_s=0.002,
        ),
        max_position_adjustment_rad=1.0,
    )
    step = controller.step(
        force_error_n=1.0,
        measured_force_n=0.0,
        stiffness_n_per_m=1000.0,
        closure_jacobian_m_per_rad=0.1,
        dt_s=0.01,
    )
    assert step.force_rate_command_n_s == pytest.approx(0.5)
    assert step.joint_velocity_command_rad_s == pytest.approx(0.002)
    assert step.position_adjustment_rad == pytest.approx(0.00002)
    assert step.force_rate_limited
    assert step.joint_velocity_limited
    assert controller.integral_error_n_s == pytest.approx(0.0)
