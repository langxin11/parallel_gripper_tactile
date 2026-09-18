"""验证二阶直接力矩 ADRC 调参 study 的配置和候选排序。"""

from __future__ import annotations

from pathlib import Path

import pytest

from parallel_gripper_tactile.studies.force_tracking_torque_adrc_tuning import (
    TorqueAdrcCandidate,
    load_torque_adrc_tuning_config,
)
from parallel_gripper_tactile.studies.common import StudyConfigError
from parallel_gripper_tactile.studies.protocols import (
    force_tracking_torque_adrc_tuning as package_protocol,
)


ROOT = Path(__file__).resolve().parents[1]


def test_tuning_config_filters_observer_faster_than_measurement_candidates(tmp_path: Path) -> None:
    """候选空间拒绝测量滤波带宽低于 ESO 带宽的组合。"""
    config_path = tmp_path / "tuning.yaml"
    config_path.write_text(
        """profile: profile.yaml
tasks: [step.yaml, ramp.yaml, mixed.yaml]
baseline: {measurement_filter_cutoff_hz: 40.0, controller_bandwidth_rad_s: 60.0, observer_bandwidth_ratio: 3.0}
coarse:
  materials: [medium]
  measurement_filter_cutoffs_hz: [20.0, 40.0]
  controller_bandwidths_rad_s: [60.0]
  observer_bandwidth_ratios: [2.0, 4.0]
confirm:
  materials: [medium]
  measurement_filter_cutoffs_hz: [20.0, 40.0]
  controller_bandwidths_rad_s: [60.0]
  observer_bandwidth_ratios: [2.0, 4.0]
""",
        encoding="utf-8",
    )

    config = load_torque_adrc_tuning_config(config_path)

    assert config.profile == tmp_path / "profile.yaml"
    assert [candidate.identifier for candidate in config.stage_candidates("coarse")] == [
        "fc40-wc60-ratio3",
        "fc20-wc60-ratio2",
        "fc40-wc60-ratio2",
        "fc40-wc60-ratio4",
    ]


def test_tuning_config_rejects_unknown_fields(tmp_path: Path) -> None:
    """调参 study YAML 保持严格 schema。"""
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text(
        "profile: profile.yaml\ntasks: [step.yaml]\nunexpected: true\n", encoding="utf-8"
    )

    with pytest.raises(StudyConfigError):
        load_torque_adrc_tuning_config(config_path)


def test_rank_candidates_prefers_lower_step_overshoot_with_continuous_constraints() -> None:
    """可行候选优先按阶跃超调排序，连续任务劣化超过约束则淘汰。"""
    protocol = package_protocol
    baseline = TorqueAdrcCandidate(40.0, 60.0, 3.0)
    preferred = TorqueAdrcCandidate(40.0, 40.0, 3.0)
    rejected = TorqueAdrcCandidate(60.0, 60.0, 3.0)

    def rows(
        candidate: TorqueAdrcCandidate, step_over: float, ramp: float
    ) -> list[dict[str, object]]:
        return [
            {
                "candidate_id": candidate.identifier,
                "measurement_filter_cutoff_hz": candidate.measurement_filter_cutoff_hz,
                "controller_bandwidth_rad_s": candidate.controller_bandwidth_rad_s,
                "observer_bandwidth_ratio": candidate.observer_bandwidth_ratio,
                "observer_bandwidth_rad_s": candidate.observer_bandwidth_rad_s,
                "task_name": task,
                "object_material": "medium",
                "runs": 1,
                "passed_runs": 1,
                "rmse_n_mean": rmse,
                "overshoot_ratio_mean": over,
                "settling_time_s_mean": 0.2,
                "torque_saturation_ratio_mean": 0.0,
            }
            for task, rmse, over in (
                ("step_force_tracking", 0.6, step_over),
                ("ramp_force_tracking", ramp, None),
                ("mixed_waypoint_force_tracking", 0.05, None),
            )
        ]

    ranking = protocol.rank_candidates(  # type: ignore[attr-defined]
        rows(baseline, 0.25, 0.05) + rows(preferred, 0.12, 0.052) + rows(rejected, 0.05, 0.08),
        (baseline, preferred, rejected),
        baseline=baseline,
        max_torque_saturation_ratio=0.01,
        max_ramp_rmse_ratio_to_baseline=1.10,
        max_mixed_rmse_ratio_to_baseline=1.10,
    )

    assert ranking[0]["candidate_id"] == preferred.identifier
    assert ranking[0]["feasible"] == "true"
    assert (
        next(row for row in ranking if row["candidate_id"] == rejected.identifier)["feasible"]
        == "false"
    )


def test_tuning_figures_render_with_missing_step_metrics(
    tmp_path: Path, fast_plot_render: None
) -> None:
    """coarse 或 confirm 缺少部分瞬态指标时，调参图仍保留可用性能点。"""
    protocol = package_protocol
    baseline = TorqueAdrcCandidate(40.0, 60.0, 3.0)
    candidate = TorqueAdrcCandidate(50.0, 50.0, 3.0)
    aggregates = []
    for item, step_rmse, overshoot in ((baseline, 0.50, 0.25), (candidate, 0.42, None)):
        for task_name, rmse in (
            ("step_force_tracking", step_rmse),
            ("ramp_force_tracking", 0.06),
            ("mixed_force_tracking", 0.05),
        ):
            aggregates.append(
                {
                    "candidate_id": item.identifier,
                    "measurement_filter_cutoff_hz": item.measurement_filter_cutoff_hz,
                    "controller_bandwidth_rad_s": item.controller_bandwidth_rad_s,
                    "observer_bandwidth_ratio": item.observer_bandwidth_ratio,
                    "observer_bandwidth_rad_s": item.observer_bandwidth_rad_s,
                    "task_name": task_name,
                    "object_material": "medium",
                    "runs": 1,
                    "passed_runs": 1,
                    "rmse_n_mean": rmse,
                    "overshoot_ratio_mean": overshoot if "step" in task_name else None,
                    "settling_time_s_mean": None,
                    "torque_saturation_ratio_mean": 0.0,
                }
            )
    ranking = protocol.rank_candidates(  # type: ignore[attr-defined]
        aggregates,
        (baseline, candidate),
        baseline=baseline,
        max_torque_saturation_ratio=0.01,
        max_ramp_rmse_ratio_to_baseline=1.10,
        max_mixed_rmse_ratio_to_baseline=1.10,
    )

    figures = protocol.render_study_figures(  # type: ignore[attr-defined]
        aggregates,
        ranking,
        tmp_path,
        max_torque_saturation_ratio=0.01,
        max_ramp_rmse_ratio_to_baseline=1.10,
        max_mixed_rmse_ratio_to_baseline=1.10,
    )

    assert [path.suffix for path in figures] == [".png", ".png"]
    assert all(path.is_file() and path.stat().st_size > 0 for path in figures)
