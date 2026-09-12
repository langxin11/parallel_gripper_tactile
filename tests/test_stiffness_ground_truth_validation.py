"""验证准静态等效接触刚度真值研究的配置、指标与计划。"""

from __future__ import annotations

from pathlib import Path

from hydra import compose, initialize_config_dir
import pytest

from parallel_gripper_tactile.experiments.stiffness_calibration import (
    StiffnessCalibrationTask,
    evaluate_stiffness_estimates,
)
from parallel_gripper_tactile.research import REPOSITORY_ROOT
from parallel_gripper_tactile.research.hydra_support import register_resolvers, resolved_mapping
from parallel_gripper_tactile.research.study import resolve_research_study
from parallel_gripper_tactile.studies.stiffness_ground_truth_validation import (
    StiffnessGroundTruthValidationConfig,
)


def _linear_points() -> list[dict[str, object]]:
    """构造加载／卸载均为 3000 N/m 的无迟滞平衡点。"""
    rows: list[dict[str, object]] = []
    for branch, indices in (("loading", range(5)), ("unloading", range(4, -1, -1))):
        for index in indices:
            closure = 0.010 + index * 0.0001
            rows.append(
                {
                    "branch": branch,
                    "offset_index": index,
                    "closure_m": closure,
                    "normal_force_n": 3000.0 * (closure - 0.010),
                    "estimated_stiffness_n_per_m": 3000.0,
                    "estimate_log_jitter": 0.0,
                    "reference_stiffness_n_per_m": None,
                }
            )
    return rows


def test_centered_reference_recovers_linear_stiffness() -> None:
    """分支内中心差分对线性力—闭合关系恢复无偏刚度。"""
    result = evaluate_stiffness_estimates(_linear_points())

    assert result.evaluated_points == 6
    assert result.reference_stiffness_mean_n_per_m == pytest.approx(3000.0)
    assert result.log_rmse == pytest.approx(0.0, abs=1e-12)
    assert result.relative_rmse == pytest.approx(0.0, abs=1e-12)
    assert result.underestimation_ratio == pytest.approx(0.0)
    assert result.force_increment_rmse_n == pytest.approx(0.0, abs=1e-12)
    assert result.loading_unloading_hysteresis_ratio == pytest.approx(0.0)
    assert result.passed


def test_task_requires_centered_difference_points() -> None:
    """标定任务拒绝不足三个或非递增的闭合工作点。"""
    with pytest.raises(ValueError, match="at least three"):
        StiffnessCalibrationTask(closure_offsets_m=(0.0, 0.001))
    with pytest.raises(ValueError, match="strictly increasing"):
        StiffnessCalibrationTask(closure_offsets_m=(0.0, 0.001, 0.001))


def test_formal_ground_truth_plan_is_paired_and_complete() -> None:
    """正式计划展开 27 条估计器—材料—seed 配对条件。"""
    register_resolvers()
    config_root = REPOSITORY_ROOT / "configs"
    with initialize_config_dir(version_base="1.3", config_dir=str(config_root)):
        raw = resolved_mapping(
            compose(
                config_name="study",
                overrides=["research=stiffness_ground_truth_validation/study"],
            )
        )
    resolved = resolve_research_study(raw)

    assert isinstance(resolved.domain_config, StiffnessGroundTruthValidationConfig)
    assert len(resolved.plan.conditions) == 27
    assert len({condition.condition_id for condition in resolved.plan.conditions}) == 27
    assert len({condition.pair_key for condition in resolved.plan.conditions}) == 9
    assert resolved.plan.study_kind == "stiffness_ground_truth_validation"


def test_task_file_is_repository_relative() -> None:
    """权威任务文件位于科研配置树并能独立加载。"""
    task_path = REPOSITORY_ROOT / "configs/task/stiffness_calibration/quasistatic.yaml"
    task = StiffnessCalibrationTask.load(task_path)

    assert task.closure_offsets_m[0] == 0.0
    assert task.sample_duration_s <= task.settle_duration_s
    assert Path(task_path).is_file()


def test_unexcited_estimator_is_scientific_failure() -> None:
    """初值即使恰好等于参考，未被有效激励也不能通过。"""
    points = _linear_points()
    for row in points:
        row["estimator_valid_fraction"] = 0.0
    result = evaluate_stiffness_estimates(points)
    assert not result.passed
    assert result.evaluated_points == 0


def test_unsettled_reference_is_scientific_failure() -> None:
    """未稳定的窗口不能构造参考，也不应冒充执行异常。"""
    points = _linear_points()
    for row in points:
        row["settled"] = False
    assert not evaluate_stiffness_estimates(points).passed


def test_unloading_overprediction_does_not_count_as_loading_risk() -> None:
    """卸载分支的负增量预测不混入加载低估风险。"""
    points = _linear_points()
    for row in points:
        if row["branch"] == "unloading":
            row["estimated_stiffness_n_per_m"] = 6000.0
    result = evaluate_stiffness_estimates(points)
    assert result.force_increment_underprediction_p95_n == pytest.approx(0.0, abs=1e-12)
    assert result.force_increment_rmse_n > 0.0


@pytest.mark.parametrize("offset", [float("nan"), float("inf")])
def test_nonfinite_offsets_are_rejected(offset: float) -> None:
    """非有限扫描偏移在配置阶段拒绝。"""
    with pytest.raises(ValueError):
        StiffnessCalibrationTask(closure_offsets_m=(0.0, 0.001, offset))


def test_nonuniform_quadratic_reference_uses_local_derivative() -> None:
    """非均匀网格的二次力曲线须恢复当前点导数，而非两端弦斜率。"""
    points = _linear_points()
    for row in points:
        x = 0.001 * (int(row["offset_index"]) + 1) ** 2
        row["closure_m"] = x
        row["normal_force_n"] = 1000.0 * x**2
        row["estimated_stiffness_n_per_m"] = 2000.0 * x
    assert evaluate_stiffness_estimates(points).log_rmse == pytest.approx(0.0, abs=1e-12)
