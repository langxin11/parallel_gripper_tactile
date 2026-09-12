"""验证刚度速率控制参数网格与候选排序。"""

from pathlib import Path

import pytest

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingTask
from parallel_gripper_tactile.studies.force_tracking_stiffness_rate_tuning import (
    ForceTrackingStiffnessRateTuningConfig,
)
from parallel_gripper_tactile.studies.protocols.force_tracking_stiffness_rate_tuning import (
    _platform_metrics,
    _render_confirmation_maps,
    _worst_case_candidate_metric,
    rank_candidates,
)
from parallel_gripper_tactile.studies.tabular import write_rows_csv


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


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


def test_confirmation_mode_requires_no_second_configuration_type() -> None:
    """确认阶段复用同一候选定义，并允许将网格收缩为单一冻结候选。"""
    config = ForceTrackingStiffnessRateTuningConfig(
        name="force_tracking_stiffness_rate_confirmation",
        analysis_mode="confirmation",
        tasks=(Path("step.yaml"), Path("step_250hz.yaml"), Path("step_125hz.yaml")),
        materials=("medium", "hard", "stiff"),
        kp_s_inv=(30.0,),
        max_force_rate_n_s=(70.0,),
    )

    assert len(config.candidates()) == 1
    assert len(config.conditions()) == 54


def test_force_rate_metric_deduplicates_physics_rows_at_each_control_time(
    tmp_path: Path,
) -> None:
    """事件窗口内的物理步不能把一次控制更新误判为更高力增长率。"""
    rows = [
        {
            "phase": "track_reference",
            "tracking_time_s": tracking_time_s,
            "control_time_s": control_time_s,
            "filtered_normal_force_n": force_n,
        }
        for tracking_time_s, control_time_s, force_n in (
            (0.0, 0.0, 0.0),
            (0.09, 0.1, 0.0),
            (0.1, 0.1, 1.0),
            (0.2, 0.2, 2.0),
            (0.3, 0.3, 3.0),
        )
    ]
    write_rows_csv(tmp_path / "trace.csv", rows)

    metrics = _platform_metrics(tmp_path, steady_window_s=(0.0, 0.3))

    assert metrics["max_positive_force_rate_n_s"] == pytest.approx(10.0)


def test_confirmation_figure_renders_frequency_material_matrix(tmp_path: Path) -> None:
    """单一确认候选按三种频率和材料生成可读矩阵图。"""
    task_paths = tuple(
        REPOSITORY_ROOT / f"configs/task/force_tracking/{name}.yaml"
        for name in ("step", "step_250hz", "step_125hz")
    )
    config = ForceTrackingStiffnessRateTuningConfig(
        analysis_mode="confirmation",
        tasks=task_paths,
        materials=("medium", "hard", "stiff"),
        kp_s_inv=(30.0,),
        max_force_rate_n_s=(70.0,),
    )
    aggregates = []
    for task_path in task_paths:
        task_name = ForceTrackingTask.load(task_path).name
        for material in config.materials:
            for candidate_id, controller, rmse in (
                ("pid-torque-ff", "pid-torque-ff", 0.4),
                ("kp30-rate70", "pid-stiffness-rate", 0.6),
            ):
                aggregates.append(
                    {
                        "candidate_id": candidate_id,
                        "controller_variant": controller,
                        "task_name": task_name,
                        "object_material": material,
                        "rmse_n_mean": rmse,
                        "plateau_force_std_n_mean": 0.01,
                        "overshoot_ratio_mean": 0.05,
                        "dominant_oscillation_amplitude_n_mean": 0.005,
                    }
                )

    output = _render_confirmation_maps(aggregates, config, tmp_path / "confirmation.png")

    assert output.is_file()
    assert output.stat().st_size > 0


def test_confirmation_summary_omits_oscillation_amplitude(tmp_path: Path) -> None:
    """研究总览仅保留 RMSE、平台标准差和超调，振荡幅值留给诊断图。"""
    task_paths = (REPOSITORY_ROOT / "configs/task/force_tracking/step.yaml",)
    config = ForceTrackingStiffnessRateTuningConfig(
        analysis_mode="confirmation",
        tasks=task_paths,
        materials=("medium",),
        kp_s_inv=(30.0,),
        max_force_rate_n_s=(70.0,),
    )
    task_name = ForceTrackingTask.load(task_paths[0]).name
    aggregates = [
        {
            "candidate_id": candidate_id,
            "controller_variant": candidate_id,
            "task_name": task_name,
            "object_material": "medium",
            "rmse_n_mean": rmse,
            "plateau_force_std_n_mean": 0.01,
            "overshoot_ratio_mean": 0.05,
            "dominant_oscillation_amplitude_n_mean": 0.005,
        }
        for candidate_id, rmse in (("pid-torque-ff", 0.4), ("kp30-rate70", 0.6))
    ]

    output = _render_confirmation_maps(aggregates, config, tmp_path / "summary.png")

    assert output.is_file()


def test_tuning_heatmap_uses_worst_case_across_materials() -> None:
    """多材料调优图与候选排名统一使用最坏工况口径。"""
    rows = [
        {
            "kp_s_inv": 20.0,
            "max_force_rate_n_s": 50.0,
            "rmse_n_mean": value,
        }
        for value in (0.5, 0.7)
    ]

    lookup = _worst_case_candidate_metric(rows, "rmse_n_mean")

    assert lookup[(20.0, 50.0)] == pytest.approx(0.7)


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
            "max_positive_force_rate_n_s_mean": 80.0,
            "plateau_force_std_n_mean": 0.05,
            "dominant_oscillation_amplitude_n_mean": 0.04,
        },
        {
            "candidate_id": candidates[1].identifier,
            "runs": 3,
            "passed_runs": 3,
            "rmse_n_mean": 0.5,
            "overshoot_ratio_mean": 0.04,
            "max_positive_force_rate_n_s_mean": 60.0,
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


def test_ranking_prefers_smaller_constraint_violation_when_all_candidates_fail() -> None:
    """没有可行候选时，轻微越界候选应排在低 RMSE 但严重越界候选之前。"""
    config = _config()
    candidates = config.candidates()[:2]
    aggregates = [
        {
            "candidate_id": candidates[0].identifier,
            "runs": 3,
            "passed_runs": 3,
            "rmse_n_mean": 0.4,
            "overshoot_ratio_mean": 0.13,
            "max_positive_force_rate_n_s_mean": 80.0,
            "plateau_force_std_n_mean": 0.01,
            "dominant_oscillation_amplitude_n_mean": 0.005,
        },
        {
            "candidate_id": candidates[1].identifier,
            "runs": 3,
            "passed_runs": 3,
            "rmse_n_mean": 0.6,
            "overshoot_ratio_mean": 0.105,
            "max_positive_force_rate_n_s_mean": 60.0,
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
    assert float(ranking[0]["constraint_violation"]) == pytest.approx(0.05)
