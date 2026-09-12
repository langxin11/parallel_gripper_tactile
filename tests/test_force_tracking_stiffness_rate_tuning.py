"""验证刚度速率控制参数网格与候选排序。"""

from pathlib import Path

from parallel_gripper_tactile.studies.force_tracking_stiffness_rate_tuning import (
    ForceTrackingStiffnessRateTuningConfig,
)
from parallel_gripper_tactile.studies.protocols.force_tracking_stiffness_rate_tuning import (
    rank_candidates,
)


def _config() -> ForceTrackingStiffnessRateTuningConfig:
    """返回最小但完整的 3×3 调优配置。"""
    return ForceTrackingStiffnessRateTuningConfig(
        tasks=(Path("step.yaml"),),
        materials=("stiff",),
        kp_s_inv=(10.0, 20.0, 30.0),
        max_force_rate_n_s=(30.0, 50.0, 70.0),
    )


def test_candidate_grid_adds_one_paired_performance_baseline() -> None:
    """九个候选之外仅增加一类位置式 PID 性能基线。"""
    config = _config()

    assert len(config.candidates()) == 9
    assert len(config.conditions()) == 30
    assert config.candidates()[0].identifier == "kp10-rate30"
    assert config.candidates()[-1].identifier == "kp30-rate70"
    assert sum(candidate is None for candidate, *_ in config.conditions()) == 3


def test_ranking_applies_plateau_and_overshoot_constraints_before_rmse() -> None:
    """低 RMSE 但振荡超限的候选不能排在可行候选之前。"""
    config = _config()
    candidates = config.candidates()[:2]
    aggregates = [
        {
            "candidate_id": candidates[0].identifier,
            "runs": 3,
            "passed_runs": 3,
            "rmse_n_mean": 0.4,
            "overshoot_ratio_mean": 0.05,
            "plateau_force_std_n_mean": 0.05,
            "dominant_oscillation_amplitude_n_mean": 0.04,
        },
        {
            "candidate_id": candidates[1].identifier,
            "runs": 3,
            "passed_runs": 3,
            "rmse_n_mean": 0.5,
            "overshoot_ratio_mean": 0.04,
            "plateau_force_std_n_mean": 0.01,
            "dominant_oscillation_amplitude_n_mean": 0.005,
        },
    ]

    ranking = rank_candidates(
        aggregates,
        candidates,
        max_plateau_force_std_n=0.03,
        max_overshoot_ratio=0.10,
    )

    assert ranking[0]["candidate_id"] == candidates[1].identifier
    assert ranking[0]["feasible"] == "true"
    assert ranking[1]["feasible"] == "false"
