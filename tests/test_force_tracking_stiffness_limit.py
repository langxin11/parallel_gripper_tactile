"""验证刚度位置限幅三臂研究的配对、注入与安全指标。"""

from __future__ import annotations

from hydra import compose, initialize_config_dir
import pytest

from parallel_gripper_tactile.research import REPOSITORY_ROOT
from parallel_gripper_tactile.research.hydra_support import register_resolvers, resolved_mapping
from parallel_gripper_tactile.research.study import resolve_research_study
from parallel_gripper_tactile.experiments.force_tracking import (
    ForceTrackingTask,
    validate_force_tracking_configuration,
)
from parallel_gripper_tactile.studies.force_tracking_stiffness_limit import (
    ForceTrackingStiffnessLimitConfig,
)
from parallel_gripper_tactile.studies.protocols.force_tracking_stiffness_limit import (
    configured_mode_profile,
    trace_safety_metrics,
)


def _resolved():
    """解析快速限幅研究。"""
    register_resolvers()
    with initialize_config_dir(version_base="1.3", config_dir=str(REPOSITORY_ROOT / "configs")):
        raw = resolved_mapping(
            compose(
                config_name="study",
                overrides=["research=force_tracking_stiffness_limit_pilot/study"],
            )
        )
    return resolve_research_study(raw)


def test_pilot_plan_has_three_paired_arms() -> None:
    """Step pilot 包含 27 条唯一条件和 9 个完整配对。"""
    resolved = _resolved()
    assert isinstance(resolved.domain_config, ForceTrackingStiffnessLimitConfig)
    assert len(resolved.plan.conditions) == 27
    assert len({condition.condition_id for condition in resolved.plan.conditions}) == 27
    assert len({condition.pair_key for condition in resolved.plan.conditions}) == 9


def test_oracle_profile_freezes_independent_reference() -> None:
    """oracle 臂保留限幅控制结构并冻结材料级参考。"""
    resolved = _resolved()
    config = resolved.domain_config
    assert isinstance(config, ForceTrackingStiffnessLimitConfig)
    profile = configured_mode_profile(
        resolved.profile,
        mode="oracle-k",
        material="hard",
        seed=0,
        oracle_values=config.oracle_stiffness_n_per_m,
        force_rate_limit_n_s=config.position_limit_force_rate_n_s,
    )
    stiffness = profile.normal_force.stiffness
    assert stiffness.position_limit_enabled
    assert stiffness.method == "window_linear"
    assert stiffness.initial_n_per_m == pytest.approx(config.oracle_stiffness_n_per_m["hard"])
    assert stiffness.min_delta_closure_m == pytest.approx(1.0)
    assert stiffness.position_limit_force_rate_n_s == pytest.approx(50.0)


def test_no_limit_preserves_torque_feedforward() -> None:
    """no-limit 只移除位置限幅，不移除机构力矩前馈。"""
    resolved = _resolved()
    config = resolved.domain_config
    assert isinstance(config, ForceTrackingStiffnessLimitConfig)
    profile = configured_mode_profile(
        resolved.profile,
        mode="no-limit",
        material="hard",
        seed=0,
        oracle_values=config.oracle_stiffness_n_per_m,
        force_rate_limit_n_s=config.position_limit_force_rate_n_s,
    )
    stiffness = profile.normal_force.stiffness
    assert not stiffness.position_limit_enabled
    assert stiffness.position_feedforward_gain == pytest.approx(0.0)
    assert stiffness.torque_feedforward_gain == pytest.approx(1.0)


def test_pilot_decouples_control_and_physics_frequencies() -> None:
    """快速研究以 250 Hz 外环控制 500 Hz MuJoCo 物理步进。"""
    resolved = _resolved()
    config = resolved.domain_config
    assert isinstance(config, ForceTrackingStiffnessLimitConfig)
    task = ForceTrackingTask.load(config.tasks[0])
    model = validate_force_tracking_configuration(resolved.profile, task=task)
    assert task.control_period_s == pytest.approx(0.004)
    assert model.opt.timestep == pytest.approx(0.002)


def test_trace_safety_metrics_use_contact_window_and_positive_rate() -> None:
    """接触峰值限于指定窗口，增长率只保留正方向。"""
    rows = [
        {"time_s": 1.0, "phase": "contact_settle", "filtered_normal_force_n": 0.4},
        {"time_s": 1.1, "phase": "track_reference", "filtered_normal_force_n": 0.8},
        {"time_s": 1.2, "phase": "track_reference", "filtered_normal_force_n": 0.6},
        {"time_s": 1.3, "phase": "track_reference", "filtered_normal_force_n": 1.1},
    ]
    metrics = trace_safety_metrics(rows, contact_time_s=1.0, contact_window_s=0.2)
    assert metrics["contact_window_peak_force_n"] == pytest.approx(0.8)
    assert metrics["max_positive_force_rate_n_s"] == pytest.approx(5.0)
