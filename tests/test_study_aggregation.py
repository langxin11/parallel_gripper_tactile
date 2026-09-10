"""共享声明式聚合模块与研究脚本聚合函数的黄金回归测试。"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import pytest

from parallel_gripper_tactile.studies.aggregation import (
    aggregate_records,
    bool_rate,
    bool_sum,
    count,
    finite_mean,
    finite_std,
    first,
    first_bool,
    key,
    minimum,
    nan_mean,
    nan_std,
    optional_mean,
    optional_std,
    plain_mean,
    threshold_count,
)


ROOT = Path(__file__).resolve().parents[1]


def _protocol_module(stem: str) -> object:
    """按既有测试的方式从 scripts/experiments 加载脚本模块。"""
    path = ROOT / "scripts" / "experiments" / f"{stem}.py"
    spec = importlib.util.spec_from_file_location(f"{stem}_aggregation_protocol", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------------------
# 共享模块单元测试
# ---------------------------------------------------------------------------


def test_group_order_follows_first_appearance_and_key_is_stringified() -> None:
    """输出行序为分组首次出现顺序，分组键统一字符串化。"""
    rows = [
        {"g": "zz-group", "v": 1.0},
        {"g": "aa-group", "v": 2.0},
        {"g": "zz-group", "v": 3.0},
    ]
    aggregates = aggregate_records(rows, keys=("g",), columns=(key("g"), count(), plain_mean("v")))
    assert [row["g"] for row in aggregates] == ["zz-group", "aa-group"]
    assert [row["runs"] for row in aggregates] == [2, 1]
    assert aggregates[0]["v_mean"] == pytest.approx(2.0)
    # 整数键同样按 str() 语义字符串化后输出。
    numeric_keyed = aggregate_records([{"g": 7, "v": 1.0}], keys=("g",), columns=(key("g"),))
    assert numeric_keyed[0]["g"] == "7"


def test_multiple_keys_interleaved_with_passthrough_columns() -> None:
    """键列可穿插在透传列之间，列序与声明顺序一致。"""
    rows = [{"k1": "a", "k2": "step", "p": 3, "v": 1.0}]
    aggregates = aggregate_records(
        rows,
        keys=("k1", "k2"),
        columns=(first("p"), key("k1"), key("k2"), count()),
    )
    assert list(aggregates[0]) == ["p", "k1", "k2", "runs"]
    assert aggregates[0]["p"] == 3
    assert aggregates[0]["k1"] == "a"
    assert aggregates[0]["k2"] == "step"


def test_count_bool_sum_and_first_passthrough() -> None:
    """行数与真值计数为整数，首行透传保留原始值与类型。"""
    rows = [
        {"g": "a", "passed": True, "flag": False, "n": 2, "x": 0.5},
        {"g": "a", "passed": False, "flag": True, "n": 4, "x": None},
    ]
    aggregate = aggregate_records(
        rows,
        keys=("g",),
        columns=(
            count("runs"),
            bool_sum("passed", "passed_runs"),
            bool_sum("flag", "flag_runs"),
            first("n"),
            first("x"),
            first_bool("passed"),
        ),
    )[0]
    assert aggregate["runs"] == 2
    assert aggregate["passed_runs"] == 1
    assert aggregate["flag_runs"] == 1
    assert aggregate["n"] == 2
    assert aggregate["x"] == 0.5
    assert aggregate["passed"] is True


def test_finite_stats_drop_none_nan_and_inf() -> None:
    """有限值统计剔除 None、NaN 与 inf，空组均值为 None、标准差不足两个样本为 None。"""
    rows = [
        {"g": "a", "v": 1.0},
        {"g": "a", "v": math.nan},
        {"g": "a", "v": math.inf},
        {"g": "a", "v": None},
        {"g": "a", "v": 3.0},
        {"g": "b", "v": math.nan},
        {"g": "b", "v": None},
        {"g": "c", "v": 2.5},
    ]
    aggregates = aggregate_records(
        rows,
        keys=("g",),
        columns=(key("g"), finite_mean("v"), finite_std("v")),
    )
    by_group = {row["g"]: row for row in aggregates}
    assert by_group["a"]["v_mean"] == pytest.approx(2.0)
    assert by_group["a"]["v_std"] == pytest.approx(math.sqrt(2.0))
    assert by_group["b"]["v_mean"] is None
    assert by_group["b"]["v_std"] is None
    assert by_group["c"]["v_mean"] == pytest.approx(2.5)
    assert by_group["c"]["v_std"] is None


def test_optional_stats_only_drop_none_and_propagate_nan() -> None:
    """仅过滤 None 的统计不滤 NaN 或 inf，NaN 像 fmean 一样传播为 NaN。"""
    from statistics import fmean

    rows = [
        {"g": "a", "v": 1.0},
        {"g": "a", "v": None},
        {"g": "a", "v": 3.0},
        {"g": "b", "v": math.nan},
        {"g": "b", "v": 2.0},
    ]
    aggregates = aggregate_records(
        rows,
        keys=("g",),
        columns=(key("g"), optional_mean("v"), optional_std("v")),
    )
    by_group = {row["g"]: row for row in aggregates}
    assert by_group["a"]["v_mean"] == pytest.approx(fmean([1.0, 3.0]))
    assert by_group["a"]["v_std"] == pytest.approx(math.sqrt(2.0))
    assert math.isnan(by_group["b"]["v_mean"])
    assert math.isnan(by_group["b"]["v_std"])
    # inf 不被过滤，参与统计。
    inf_aggregate = aggregate_records(
        [{"g": "a", "v": math.inf}, {"g": "a", "v": 2.0}],
        keys=("g",),
        columns=(optional_mean("v"),),
    )[0]
    assert inf_aggregate["v_mean"] == math.inf


def test_plain_mean_matches_fmean_and_propagates_nan() -> None:
    """无过滤均值与 statistics.fmean 数值一致，NaN 传播为 NaN。"""
    from statistics import fmean

    values = [0.1, 0.25, 1.0 / 3.0, 2.75, 10.5]
    rows = [{"g": "a", "v": value} for value in values]
    aggregate = aggregate_records(rows, keys=("g",), columns=(plain_mean("v"),))[0]
    assert aggregate["v_mean"] == pytest.approx(fmean(values))
    nan_aggregate = aggregate_records(
        [{"g": "a", "v": math.nan}, *rows], keys=("g",), columns=(plain_mean("v"),)
    )[0]
    assert math.isnan(nan_aggregate["v_mean"])


def test_nan_stats_follow_numpy_nan_semantics() -> None:
    """NaN 感知统计忽略 None 与 NaN、保留 inf，不足两个有效值时输出 NaN。"""
    rows = [
        {"g": "a", "v": None},
        {"g": "a", "v": 0.4},
        {"g": "a", "v": math.nan},
        {"g": "a", "v": 0.8},
        {"g": "b", "v": None},
        {"g": "b", "v": None},
        {"g": "c", "v": 1.5},
        {"g": "d", "v": math.inf},
        {"g": "d", "v": 1.0},
    ]
    aggregates = aggregate_records(
        rows, keys=("g",), columns=(key("g"), nan_mean("v"), nan_std("v"))
    )
    by_group = {row["g"]: row for row in aggregates}
    assert by_group["a"]["v_mean"] == pytest.approx(0.6)
    assert by_group["a"]["v_std"] == pytest.approx(math.sqrt(0.08))
    assert math.isnan(by_group["b"]["v_mean"])
    assert math.isnan(by_group["b"]["v_std"])
    assert by_group["c"]["v_mean"] == pytest.approx(1.5)
    assert math.isnan(by_group["c"]["v_std"])
    assert by_group["d"]["v_mean"] == math.inf
    assert math.isnan(by_group["d"]["v_std"])


def test_missing_metric_columns_are_treated_as_none() -> None:
    """输入行整体缺失的指标列按全 None 处理，不因列缺失而报错。"""
    rows = [
        {"g": "a", "passed": True, "rmse_n": 0.2},
        {"g": "a", "passed": False, "rmse_n": 0.4},
    ]
    aggregate = aggregate_records(
        rows,
        keys=("g",),
        columns=(
            key("g"),
            count("runs"),
            bool_sum("passed", "passed_runs"),
            finite_mean("rmse_n"),
            nan_mean("rise_time_s"),
            nan_std("rise_time_s"),
            optional_mean("settling_time_s"),
        ),
    )[0]
    assert aggregate["runs"] == 2
    assert aggregate["passed_runs"] == 1
    assert aggregate["rmse_n_mean"] == pytest.approx(0.3)
    assert math.isnan(aggregate["rise_time_s_mean"])
    assert math.isnan(aggregate["rise_time_s_std"])
    assert aggregate["settling_time_s_mean"] is None


def test_bool_rate_and_zero_when_gate() -> None:
    """真值占比等于 sum/len，zero_when 门控命中时输出固定 0.0。"""
    rows = [
        {"g": "positive", "e": True, "d": True},
        {"g": "positive", "e": True, "d": False},
        {"g": "negative", "e": False, "d": True},
        {"g": "negative", "e": False, "d": False},
    ]
    aggregates = aggregate_records(
        rows,
        keys=("g",),
        columns=(
            key("g"),
            first_bool("e"),
            bool_rate("d", "detection_rate"),
            bool_rate("d", "false_positive_rate", zero_when=("e", True)),
        ),
    )
    by_group = {row["g"]: row for row in aggregates}
    assert by_group["positive"]["e"] is True
    assert by_group["positive"]["detection_rate"] == pytest.approx(0.5)
    assert by_group["positive"]["false_positive_rate"] == 0.0
    assert by_group["negative"]["detection_rate"] == pytest.approx(0.5)
    assert by_group["negative"]["false_positive_rate"] == pytest.approx(0.5)


def test_threshold_count_excludes_nan_and_null() -> None:
    """阈值计数为闭区间比较，NaN 与 None 均不满足阈值。"""
    rows = [
        {"g": "a", "r": 0.95},
        {"g": "a", "r": 0.9},
        {"g": "a", "r": 0.85},
        {"g": "a", "r": math.nan},
        {"g": "a", "r": None},
        {"g": "b", "r": math.inf},
    ]
    aggregates = aggregate_records(
        rows,
        keys=("g",),
        columns=(
            key("g"),
            threshold_count("r", "complete_runs", at_least=0.9),
            minimum("r", "r_min"),
        ),
    )
    by_group = {row["g"]: row for row in aggregates}
    assert by_group["a"]["complete_runs"] == 2
    assert by_group["a"]["r_min"] == pytest.approx(0.85)
    assert by_group["b"]["complete_runs"] == 1
    assert by_group["b"]["r_min"] == math.inf


def test_empty_rows_return_empty_list_and_key_count_is_validated() -> None:
    """空输入返回空列表，分组键数量必须在 1 到 3 之间。"""
    assert aggregate_records([], keys=("g",), columns=(key("g"), count())) == []
    with pytest.raises(ValueError):
        aggregate_records([{"g": "a"}], keys=(), columns=(count(),))
    with pytest.raises(ValueError):
        aggregate_records(
            [{"a": 1, "b": 2, "c": 3, "d": 4}],
            keys=("a", "b", "c", "d"),
            columns=(count(),),
        )


# ---------------------------------------------------------------------------
# 研究脚本聚合函数的黄金回归
# ---------------------------------------------------------------------------

_SAMPLE_ROWS_JSON: dict[str, str] = {
    "dm_admittance_tuning": (
        r"""
        [
    {"candidate_id": "zz-heavy", "mass_kg": 2.0, "damping_ns_m": 12.0, "stiffness_n_m": 900.0, "filter_cutoff_hz": 8.0, "velocity_limit_rad_s": 0.9, "approach_velocity_rad_s": 0.35, "contact_stable_time_s": 0.12, "contact_transition_time_s": 0.25, "approach_feedforward_force_n": 0.8, "passed": true, "force_tracking_ratio": 0.95, "rmse_n": 0.21, "mae_n": 0.12, "peak_abs_error_n": 0.55, "final_error_n": 0.03, "raw_rmse_n": 0.3, "raw_mae_n": 0.18, "raw_peak_abs_error_n": 0.8},
    {"candidate_id": "aa-light", "mass_kg": 0.8, "damping_ns_m": 6.5, "stiffness_n_m": 350.0, "filter_cutoff_hz": 12.0, "velocity_limit_rad_s": 1.2, "approach_velocity_rad_s": 0.5, "contact_stable_time_s": 0.08, "contact_transition_time_s": 0.3, "approach_feedforward_force_n": 1.1, "passed": false, "force_tracking_ratio": 1.0, "rmse_n": NaN, "mae_n": NaN, "peak_abs_error_n": null, "final_error_n": Infinity, "raw_rmse_n": null, "raw_mae_n": NaN, "raw_peak_abs_error_n": null},
    {"candidate_id": "zz-heavy", "mass_kg": 2.0, "damping_ns_m": 12.0, "stiffness_n_m": 900.0, "filter_cutoff_hz": 8.0, "velocity_limit_rad_s": 0.9, "approach_velocity_rad_s": 0.35, "contact_stable_time_s": 0.12, "contact_transition_time_s": 0.25, "approach_feedforward_force_n": 0.8, "passed": false, "force_tracking_ratio": 0.9, "rmse_n": null, "mae_n": NaN, "peak_abs_error_n": Infinity, "final_error_n": 0.07, "raw_rmse_n": NaN, "raw_mae_n": 0.2, "raw_peak_abs_error_n": null},
    {"candidate_id": "aa-light", "mass_kg": 0.8, "damping_ns_m": 6.5, "stiffness_n_m": 350.0, "filter_cutoff_hz": 12.0, "velocity_limit_rad_s": 1.2, "approach_velocity_rad_s": 0.5, "contact_stable_time_s": 0.08, "contact_transition_time_s": 0.3, "approach_feedforward_force_n": 1.1, "passed": false, "force_tracking_ratio": 0.5, "rmse_n": 0.4, "mae_n": 0.31, "peak_abs_error_n": null, "final_error_n": null, "raw_rmse_n": null, "raw_mae_n": null, "raw_peak_abs_error_n": null},
    {"candidate_id": "zz-heavy", "mass_kg": 2.0, "damping_ns_m": 12.0, "stiffness_n_m": 900.0, "filter_cutoff_hz": 8.0, "velocity_limit_rad_s": 0.9, "approach_velocity_rad_s": 0.35, "contact_stable_time_s": 0.12, "contact_transition_time_s": 0.25, "approach_feedforward_force_n": 0.8, "passed": true, "force_tracking_ratio": 0.85, "rmse_n": 0.25, "mae_n": 0.14, "peak_abs_error_n": 0.6, "final_error_n": 0.02, "raw_rmse_n": 0.28, "raw_mae_n": 0.22, "raw_peak_abs_error_n": 0.75}
        ]
        """
    ),
    "force_tracking_ablation": (
        r"""
        [
    {"controller_variant": "pid-only", "object_material": "stiff", "passed": true, "contact_time_s": 0.9, "tracking_start_time_s": 1.2, "rmse_n": 0.2, "mae_n": 0.1, "peak_abs_error_n": 0.5, "mean_error_n": 0.02, "final_error_n": 0.01, "torque_saturation_ratio": 0.0, "position_saturation_ratio": 0.0, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": null, "overshoot_ratio": 0.1, "settling_time_s": null, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"controller_variant": "full", "object_material": "hard", "passed": true, "contact_time_s": 1.4, "tracking_start_time_s": 1.8, "rmse_n": 0.31, "mae_n": 0.13, "peak_abs_error_n": 0.6, "mean_error_n": 0.01, "final_error_n": 0.02, "torque_saturation_ratio": 0.1, "position_saturation_ratio": 0.0, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": 0.2, "overshoot_ratio": null, "settling_time_s": 0.5, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"controller_variant": "pid-only", "object_material": "stiff", "passed": false, "contact_time_s": 1.0, "tracking_start_time_s": 1.25, "rmse_n": 0.4, "mae_n": NaN, "peak_abs_error_n": NaN, "mean_error_n": -0.01, "final_error_n": 0.03, "torque_saturation_ratio": 0.05, "position_saturation_ratio": 0.0, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": 0.25, "overshoot_ratio": 0.2, "settling_time_s": null, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"controller_variant": "window-quadratic", "object_material": "medium", "passed": false, "contact_time_s": 0.75, "tracking_start_time_s": 1.05, "rmse_n": 0.44, "mae_n": 0.2, "peak_abs_error_n": 0.52, "mean_error_n": -0.02, "final_error_n": -0.01, "torque_saturation_ratio": 0.0, "position_saturation_ratio": 0.12, "mean_estimated_stiffness_n_per_m": 520.0, "rise_time_s": 0.3, "overshoot_ratio": null, "settling_time_s": 0.7, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"controller_variant": "pid-only", "object_material": "stiff", "passed": true, "contact_time_s": 1.1, "tracking_start_time_s": 1.3, "rmse_n": 0.6, "mae_n": 0.15, "peak_abs_error_n": 0.7, "mean_error_n": 0.04, "final_error_n": 0.02, "torque_saturation_ratio": Infinity, "position_saturation_ratio": 0.1, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": 0.35, "overshoot_ratio": null, "settling_time_s": 0.8, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"controller_variant": "full", "object_material": "hard", "passed": true, "contact_time_s": 1.5, "tracking_start_time_s": 1.9, "rmse_n": Infinity, "mae_n": NaN, "peak_abs_error_n": NaN, "mean_error_n": 0.01, "final_error_n": 0.05, "torque_saturation_ratio": 0.2, "position_saturation_ratio": 0.05, "mean_estimated_stiffness_n_per_m": 500.0, "rise_time_s": 0.4, "overshoot_ratio": null, "settling_time_s": 0.9, "sensor_noise_seed": 7, "run_directory": "runs/demo"}
        ]
        """
    ),
    "force_tracking_controller_comparison": (
        r"""
        [
    {"controller_variant": "pid-only", "task_name": "step_force_tracking", "object_material": "stiff", "passed": true, "contact_time_s": 0.9, "tracking_start_time_s": 1.2, "rmse_n": 0.2, "mae_n": 0.1, "peak_abs_error_n": 0.5, "mean_error_n": 0.02, "final_error_n": 0.01, "torque_saturation_ratio": 0.0, "position_saturation_ratio": 0.0, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": null, "overshoot_ratio": 0.1, "settling_time_s": null, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"controller_variant": "full", "task_name": "ramp_force_tracking", "object_material": "hard", "passed": true, "contact_time_s": 1.4, "tracking_start_time_s": 1.8, "rmse_n": 0.31, "mae_n": 0.13, "peak_abs_error_n": 0.6, "mean_error_n": 0.01, "final_error_n": 0.02, "torque_saturation_ratio": 0.1, "position_saturation_ratio": 0.0, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": 0.2, "overshoot_ratio": null, "settling_time_s": 0.5, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"controller_variant": "pid-only", "task_name": "step_force_tracking", "object_material": "stiff", "passed": false, "contact_time_s": 1.0, "tracking_start_time_s": 1.25, "rmse_n": 0.4, "mae_n": NaN, "peak_abs_error_n": NaN, "mean_error_n": -0.01, "final_error_n": 0.03, "torque_saturation_ratio": 0.05, "position_saturation_ratio": 0.0, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": 0.25, "overshoot_ratio": 0.2, "settling_time_s": null, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"controller_variant": "window-quadratic", "task_name": "step_force_tracking", "object_material": "medium", "passed": false, "contact_time_s": 0.75, "tracking_start_time_s": 1.05, "rmse_n": 0.44, "mae_n": 0.2, "peak_abs_error_n": 0.52, "mean_error_n": -0.02, "final_error_n": -0.01, "torque_saturation_ratio": 0.0, "position_saturation_ratio": 0.12, "mean_estimated_stiffness_n_per_m": 520.0, "rise_time_s": 0.3, "overshoot_ratio": null, "settling_time_s": 0.7, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"controller_variant": "pid-only", "task_name": "step_force_tracking", "object_material": "stiff", "passed": true, "contact_time_s": 1.1, "tracking_start_time_s": 1.3, "rmse_n": 0.6, "mae_n": 0.15, "peak_abs_error_n": 0.7, "mean_error_n": 0.04, "final_error_n": 0.02, "torque_saturation_ratio": Infinity, "position_saturation_ratio": 0.1, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": 0.35, "overshoot_ratio": null, "settling_time_s": 0.8, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"controller_variant": "full", "task_name": "ramp_force_tracking", "object_material": "hard", "passed": true, "contact_time_s": 1.5, "tracking_start_time_s": 1.9, "rmse_n": Infinity, "mae_n": NaN, "peak_abs_error_n": NaN, "mean_error_n": 0.01, "final_error_n": 0.05, "torque_saturation_ratio": 0.2, "position_saturation_ratio": 0.05, "mean_estimated_stiffness_n_per_m": 500.0, "rise_time_s": 0.4, "overshoot_ratio": null, "settling_time_s": 0.9, "sensor_noise_seed": 7, "run_directory": "runs/demo"}
        ]
        """
    ),
    "force_tracking_stiffness_estimator_comparison": (
        r"""
        [
    {"stiffness_estimator_method": "pid-only", "task_name": "step_force_tracking", "object_material": "stiff", "passed": true, "contact_time_s": 0.9, "tracking_start_time_s": 1.2, "rmse_n": 0.2, "mae_n": 0.1, "peak_abs_error_n": 0.5, "mean_error_n": 0.02, "final_error_n": 0.01, "torque_saturation_ratio": 0.0, "position_saturation_ratio": 0.0, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": null, "overshoot_ratio": 0.1, "settling_time_s": null, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"stiffness_estimator_method": "full", "task_name": "ramp_force_tracking", "object_material": "hard", "passed": true, "contact_time_s": 1.4, "tracking_start_time_s": 1.8, "rmse_n": 0.31, "mae_n": 0.13, "peak_abs_error_n": 0.6, "mean_error_n": 0.01, "final_error_n": 0.02, "torque_saturation_ratio": 0.1, "position_saturation_ratio": 0.0, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": 0.2, "overshoot_ratio": null, "settling_time_s": 0.5, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"stiffness_estimator_method": "pid-only", "task_name": "step_force_tracking", "object_material": "stiff", "passed": false, "contact_time_s": 1.0, "tracking_start_time_s": 1.25, "rmse_n": 0.4, "mae_n": NaN, "peak_abs_error_n": NaN, "mean_error_n": -0.01, "final_error_n": 0.03, "torque_saturation_ratio": 0.05, "position_saturation_ratio": 0.0, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": 0.25, "overshoot_ratio": 0.2, "settling_time_s": null, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"stiffness_estimator_method": "window-quadratic", "task_name": "step_force_tracking", "object_material": "medium", "passed": false, "contact_time_s": 0.75, "tracking_start_time_s": 1.05, "rmse_n": 0.44, "mae_n": 0.2, "peak_abs_error_n": 0.52, "mean_error_n": -0.02, "final_error_n": -0.01, "torque_saturation_ratio": 0.0, "position_saturation_ratio": 0.12, "mean_estimated_stiffness_n_per_m": 520.0, "rise_time_s": 0.3, "overshoot_ratio": null, "settling_time_s": 0.7, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"stiffness_estimator_method": "pid-only", "task_name": "step_force_tracking", "object_material": "stiff", "passed": true, "contact_time_s": 1.1, "tracking_start_time_s": 1.3, "rmse_n": 0.6, "mae_n": 0.15, "peak_abs_error_n": 0.7, "mean_error_n": 0.04, "final_error_n": 0.02, "torque_saturation_ratio": Infinity, "position_saturation_ratio": 0.1, "mean_estimated_stiffness_n_per_m": NaN, "rise_time_s": 0.35, "overshoot_ratio": null, "settling_time_s": 0.8, "sensor_noise_seed": 7, "run_directory": "runs/demo"},
    {"stiffness_estimator_method": "full", "task_name": "ramp_force_tracking", "object_material": "hard", "passed": true, "contact_time_s": 1.5, "tracking_start_time_s": 1.9, "rmse_n": Infinity, "mae_n": NaN, "peak_abs_error_n": NaN, "mean_error_n": 0.01, "final_error_n": 0.05, "torque_saturation_ratio": 0.2, "position_saturation_ratio": 0.05, "mean_estimated_stiffness_n_per_m": 500.0, "rise_time_s": 0.4, "overshoot_ratio": null, "settling_time_s": 0.9, "sensor_noise_seed": 7, "run_directory": "runs/demo"}
        ]
        """
    ),
    "force_tracking_torque_adrc_tuning": (
        r"""
        [
    {"candidate_id": "zz-wide", "measurement_filter_cutoff_hz": 15.0, "controller_bandwidth_rad_s": 40.0, "observer_bandwidth_ratio": 8.0, "observer_bandwidth_rad_s": 320.0, "task_name": "step_force_tracking", "object_material": "medium", "passed": true, "rmse_n": 0.5, "overshoot_ratio": null, "settling_time_s": NaN, "torque_saturation_ratio": 0.0},
    {"candidate_id": "aa-base", "measurement_filter_cutoff_hz": 10.0, "controller_bandwidth_rad_s": 24.0, "observer_bandwidth_ratio": 10.0, "observer_bandwidth_rad_s": 240.0, "task_name": "step_force_tracking", "object_material": "medium", "passed": false, "rmse_n": 0.32, "overshoot_ratio": 0.12, "settling_time_s": 0.9, "torque_saturation_ratio": Infinity},
    {"candidate_id": "zz-wide", "measurement_filter_cutoff_hz": 15.0, "controller_bandwidth_rad_s": 40.0, "observer_bandwidth_ratio": 8.0, "observer_bandwidth_rad_s": 320.0, "task_name": "step_force_tracking", "object_material": "medium", "passed": false, "rmse_n": 0.7, "overshoot_ratio": 0.25, "settling_time_s": 1.1, "torque_saturation_ratio": 0.02},
    {"candidate_id": "zz-wide", "measurement_filter_cutoff_hz": 15.0, "controller_bandwidth_rad_s": 40.0, "observer_bandwidth_ratio": 8.0, "observer_bandwidth_rad_s": 320.0, "task_name": "ramp_force_tracking", "object_material": "medium", "passed": true, "rmse_n": 0.08, "overshoot_ratio": null, "settling_time_s": null, "torque_saturation_ratio": null},
    {"candidate_id": "aa-base", "measurement_filter_cutoff_hz": 10.0, "controller_bandwidth_rad_s": 24.0, "observer_bandwidth_ratio": 10.0, "observer_bandwidth_rad_s": 240.0, "task_name": "step_force_tracking", "object_material": "medium", "passed": true, "rmse_n": 0.4, "overshoot_ratio": null, "settling_time_s": 1.2, "torque_saturation_ratio": null}
        ]
        """
    ),
    "robotiq_discrete_force": (
        r"""
        [
    {"controller_variant": "fixed-step", "passed": true, "safety_violated": false, "safety_violation_duration_s": 0.0, "release_count": 3, "action_count": 42, "average_nonzero_action_step": 0.012, "total_command_movement": 0.55, "reverse_count": 1, "oscillation_count": 2, "settling_time_s": 0.42, "hold_ratio": 0.85, "steady_force_error_n": 0.06, "rmse_n": 0.11, "peak_overshoot_n": 0.32, "prediction_mae_n": 0.21},
    {"controller_variant": "adaptive-deadband", "passed": false, "safety_violated": true, "safety_violation_duration_s": 1.5, "release_count": 2, "action_count": 37, "average_nonzero_action_step": 0.015, "total_command_movement": 0.61, "reverse_count": 0, "oscillation_count": 3, "settling_time_s": null, "hold_ratio": 0.78, "steady_force_error_n": 0.09, "rmse_n": 0.13, "peak_overshoot_n": 0.41, "prediction_mae_n": null},
    {"controller_variant": "fixed-step", "passed": false, "safety_violated": true, "safety_violation_duration_s": 0.5, "release_count": 4, "action_count": 51, "average_nonzero_action_step": 0.011, "total_command_movement": 0.7, "reverse_count": 2, "oscillation_count": 1, "settling_time_s": null, "hold_ratio": 0.81, "steady_force_error_n": 0.04, "rmse_n": 0.09, "peak_overshoot_n": 0.28, "prediction_mae_n": null},
    {"controller_variant": "fixed-step", "passed": true, "safety_violated": false, "safety_violation_duration_s": 0.0, "release_count": 5, "action_count": 48, "average_nonzero_action_step": 0.013, "total_command_movement": 0.58, "reverse_count": 1, "oscillation_count": 2, "settling_time_s": 0.56, "hold_ratio": 0.88, "steady_force_error_n": 0.05, "rmse_n": 0.1, "peak_overshoot_n": 0.3, "prediction_mae_n": null},
    {"controller_variant": "adaptive-deadband", "passed": true, "safety_violated": false, "safety_violation_duration_s": 0.0, "release_count": 2, "action_count": 40, "average_nonzero_action_step": 0.014, "total_command_movement": 0.66, "reverse_count": 1, "oscillation_count": 4, "settling_time_s": null, "hold_ratio": 0.8, "steady_force_error_n": 0.07, "rmse_n": 0.12, "peak_overshoot_n": 0.38, "prediction_mae_n": null}
        ]
        """
    ),
    "friction_estimation_local_slip": (
        r"""
        [
    {"scenario": "positive_slip", "expect_local_slip": true, "local_slip_detected": true, "detection_lead_s": 0.12, "local_estimate_ratio": 0.8, "validation_passed": true, "control_candidate_qualified": true},
    {"scenario": "negative_noslip", "expect_local_slip": false, "local_slip_detected": false, "detection_lead_s": null, "local_estimate_ratio": null, "validation_passed": true, "control_candidate_qualified": false},
    {"scenario": "positive_slip", "expect_local_slip": true, "local_slip_detected": true, "detection_lead_s": null, "local_estimate_ratio": 0.9, "validation_passed": false, "control_candidate_qualified": false},
    {"scenario": "single_probe", "expect_local_slip": true, "local_slip_detected": true, "detection_lead_s": 0.05, "local_estimate_ratio": null, "validation_passed": false, "control_candidate_qualified": true},
    {"scenario": "positive_slip", "expect_local_slip": true, "local_slip_detected": false, "detection_lead_s": 0.18, "local_estimate_ratio": null, "validation_passed": true, "control_candidate_qualified": false},
    {"scenario": "negative_noslip", "expect_local_slip": false, "local_slip_detected": true, "detection_lead_s": null, "local_estimate_ratio": 1.1, "validation_passed": false, "control_candidate_qualified": false}
        ]
        """
    ),
}

_SAMPLE_ROWS = {name: json.loads(text) for name, text in _SAMPLE_ROWS_JSON.items()}

_GOLDEN_JSON: dict[str, str] = {
    "dm_admittance_tuning": (
        r"""
        [
    {"candidate_id": "zz-heavy", "mass_kg": 2.0, "damping_ns_m": 12.0, "stiffness_n_m": 900.0, "filter_cutoff_hz": 8.0, "velocity_limit_rad_s": 0.9, "approach_velocity_rad_s": 0.35, "contact_stable_time_s": 0.12, "contact_transition_time_s": 0.25, "approach_feedforward_force_n": 0.8, "runs": 3, "stable_runs": 2, "complete_force_tracking_runs": 2, "force_tracking_ratio_mean": 0.9, "force_tracking_ratio_min": 0.85, "rmse_n_mean": 0.22999999999999998, "mae_n_mean": 0.13, "peak_abs_error_n_mean": 0.575, "final_error_n_mean": 0.04, "raw_rmse_n_mean": 0.29000000000000004, "raw_mae_n_mean": 0.19999999999999998, "raw_peak_abs_error_n_mean": 0.775},
    {"candidate_id": "aa-light", "mass_kg": 0.8, "damping_ns_m": 6.5, "stiffness_n_m": 350.0, "filter_cutoff_hz": 12.0, "velocity_limit_rad_s": 1.2, "approach_velocity_rad_s": 0.5, "contact_stable_time_s": 0.08, "contact_transition_time_s": 0.3, "approach_feedforward_force_n": 1.1, "runs": 2, "stable_runs": 0, "complete_force_tracking_runs": 1, "force_tracking_ratio_mean": 0.75, "force_tracking_ratio_min": 0.5, "rmse_n_mean": 0.4, "mae_n_mean": 0.31, "peak_abs_error_n_mean": null, "final_error_n_mean": null, "raw_rmse_n_mean": null, "raw_mae_n_mean": null, "raw_peak_abs_error_n_mean": null}
        ]
        """
    ),
    "force_tracking_ablation": (
        r"""
        [
    {"controller_variant": "pid-only", "object_material": "stiff", "runs": 3, "passed_runs": 2, "contact_time_s_mean": 1.0, "contact_time_s_std": 0.10000000000000003, "tracking_start_time_s_mean": 1.25, "tracking_start_time_s_std": 0.050000000000000044, "rmse_n_mean": 0.39999999999999997, "rmse_n_std": 0.19999999999999998, "mae_n_mean": 0.125, "mae_n_std": 0.03535533905932737, "peak_abs_error_n_mean": 0.6, "peak_abs_error_n_std": 0.14142135623730948, "mean_error_n_mean": 0.016666666666666666, "mean_error_n_std": 0.025166114784235832, "final_error_n_mean": 0.02, "final_error_n_std": 0.01, "torque_saturation_ratio_mean": 0.025, "torque_saturation_ratio_std": 0.035355339059327376, "position_saturation_ratio_mean": 0.03333333333333333, "position_saturation_ratio_std": 0.05773502691896258, "mean_estimated_stiffness_n_per_m_mean": null, "mean_estimated_stiffness_n_per_m_std": null, "rise_time_s_mean": 0.3, "rise_time_s_std": 0.07071067811865474, "overshoot_ratio_mean": 0.15000000000000002, "overshoot_ratio_std": 0.07071067811865477, "settling_time_s_mean": 0.8, "settling_time_s_std": NaN},
    {"controller_variant": "full", "object_material": "hard", "runs": 2, "passed_runs": 2, "contact_time_s_mean": 1.45, "contact_time_s_std": 0.07071067811865482, "tracking_start_time_s_mean": 1.85, "tracking_start_time_s_std": 0.07071067811865465, "rmse_n_mean": 0.31, "rmse_n_std": null, "mae_n_mean": 0.13, "mae_n_std": null, "peak_abs_error_n_mean": 0.6, "peak_abs_error_n_std": null, "mean_error_n_mean": 0.01, "mean_error_n_std": 0.0, "final_error_n_mean": 0.035, "final_error_n_std": 0.021213203435596427, "torque_saturation_ratio_mean": 0.15000000000000002, "torque_saturation_ratio_std": 0.07071067811865475, "position_saturation_ratio_mean": 0.025, "position_saturation_ratio_std": 0.035355339059327376, "mean_estimated_stiffness_n_per_m_mean": 500.0, "mean_estimated_stiffness_n_per_m_std": null, "rise_time_s_mean": 0.30000000000000004, "rise_time_s_std": 0.14142135623730953, "overshoot_ratio_mean": NaN, "overshoot_ratio_std": NaN, "settling_time_s_mean": 0.7, "settling_time_s_std": 0.28284271247461906},
    {"controller_variant": "window-quadratic", "object_material": "medium", "runs": 1, "passed_runs": 0, "contact_time_s_mean": 0.75, "contact_time_s_std": null, "tracking_start_time_s_mean": 1.05, "tracking_start_time_s_std": null, "rmse_n_mean": 0.44, "rmse_n_std": null, "mae_n_mean": 0.2, "mae_n_std": null, "peak_abs_error_n_mean": 0.52, "peak_abs_error_n_std": null, "mean_error_n_mean": -0.02, "mean_error_n_std": null, "final_error_n_mean": -0.01, "final_error_n_std": null, "torque_saturation_ratio_mean": 0.0, "torque_saturation_ratio_std": null, "position_saturation_ratio_mean": 0.12, "position_saturation_ratio_std": null, "mean_estimated_stiffness_n_per_m_mean": 520.0, "mean_estimated_stiffness_n_per_m_std": null, "rise_time_s_mean": 0.3, "rise_time_s_std": NaN, "overshoot_ratio_mean": NaN, "overshoot_ratio_std": NaN, "settling_time_s_mean": 0.7, "settling_time_s_std": NaN}
        ]
        """
    ),
    "force_tracking_controller_comparison": (
        r"""
        [
    {"controller_variant": "pid-only", "task_name": "step_force_tracking", "object_material": "stiff", "runs": 3, "passed_runs": 2, "contact_time_s_mean": 1.0, "contact_time_s_std": 0.10000000000000003, "tracking_start_time_s_mean": 1.25, "tracking_start_time_s_std": 0.050000000000000044, "rmse_n_mean": 0.39999999999999997, "rmse_n_std": 0.19999999999999998, "mae_n_mean": 0.125, "mae_n_std": 0.03535533905932737, "peak_abs_error_n_mean": 0.6, "peak_abs_error_n_std": 0.14142135623730948, "mean_error_n_mean": 0.016666666666666666, "mean_error_n_std": 0.025166114784235832, "final_error_n_mean": 0.02, "final_error_n_std": 0.01, "torque_saturation_ratio_mean": 0.025, "torque_saturation_ratio_std": 0.035355339059327376, "position_saturation_ratio_mean": 0.03333333333333333, "position_saturation_ratio_std": 0.05773502691896258, "mean_estimated_stiffness_n_per_m_mean": null, "mean_estimated_stiffness_n_per_m_std": null, "rise_time_s_mean": 0.3, "rise_time_s_std": 0.07071067811865474, "overshoot_ratio_mean": 0.15000000000000002, "overshoot_ratio_std": 0.07071067811865477, "settling_time_s_mean": 0.8, "settling_time_s_std": NaN},
    {"controller_variant": "full", "task_name": "ramp_force_tracking", "object_material": "hard", "runs": 2, "passed_runs": 2, "contact_time_s_mean": 1.45, "contact_time_s_std": 0.07071067811865482, "tracking_start_time_s_mean": 1.85, "tracking_start_time_s_std": 0.07071067811865465, "rmse_n_mean": 0.31, "rmse_n_std": null, "mae_n_mean": 0.13, "mae_n_std": null, "peak_abs_error_n_mean": 0.6, "peak_abs_error_n_std": null, "mean_error_n_mean": 0.01, "mean_error_n_std": 0.0, "final_error_n_mean": 0.035, "final_error_n_std": 0.021213203435596427, "torque_saturation_ratio_mean": 0.15000000000000002, "torque_saturation_ratio_std": 0.07071067811865475, "position_saturation_ratio_mean": 0.025, "position_saturation_ratio_std": 0.035355339059327376, "mean_estimated_stiffness_n_per_m_mean": 500.0, "mean_estimated_stiffness_n_per_m_std": null, "rise_time_s_mean": 0.30000000000000004, "rise_time_s_std": 0.14142135623730953, "overshoot_ratio_mean": NaN, "overshoot_ratio_std": NaN, "settling_time_s_mean": 0.7, "settling_time_s_std": 0.28284271247461906},
    {"controller_variant": "window-quadratic", "task_name": "step_force_tracking", "object_material": "medium", "runs": 1, "passed_runs": 0, "contact_time_s_mean": 0.75, "contact_time_s_std": null, "tracking_start_time_s_mean": 1.05, "tracking_start_time_s_std": null, "rmse_n_mean": 0.44, "rmse_n_std": null, "mae_n_mean": 0.2, "mae_n_std": null, "peak_abs_error_n_mean": 0.52, "peak_abs_error_n_std": null, "mean_error_n_mean": -0.02, "mean_error_n_std": null, "final_error_n_mean": -0.01, "final_error_n_std": null, "torque_saturation_ratio_mean": 0.0, "torque_saturation_ratio_std": null, "position_saturation_ratio_mean": 0.12, "position_saturation_ratio_std": null, "mean_estimated_stiffness_n_per_m_mean": 520.0, "mean_estimated_stiffness_n_per_m_std": null, "rise_time_s_mean": 0.3, "rise_time_s_std": NaN, "overshoot_ratio_mean": NaN, "overshoot_ratio_std": NaN, "settling_time_s_mean": 0.7, "settling_time_s_std": NaN}
        ]
        """
    ),
    "force_tracking_stiffness_estimator_comparison": (
        r"""
        [
    {"stiffness_estimator_method": "pid-only", "task_name": "step_force_tracking", "object_material": "stiff", "runs": 3, "passed_runs": 2, "contact_time_s_mean": 1.0, "contact_time_s_std": 0.10000000000000003, "tracking_start_time_s_mean": 1.25, "tracking_start_time_s_std": 0.050000000000000044, "rmse_n_mean": 0.39999999999999997, "rmse_n_std": 0.19999999999999998, "mae_n_mean": 0.125, "mae_n_std": 0.03535533905932737, "peak_abs_error_n_mean": 0.6, "peak_abs_error_n_std": 0.14142135623730948, "mean_error_n_mean": 0.016666666666666666, "mean_error_n_std": 0.025166114784235832, "final_error_n_mean": 0.02, "final_error_n_std": 0.01, "torque_saturation_ratio_mean": 0.025, "torque_saturation_ratio_std": 0.035355339059327376, "position_saturation_ratio_mean": 0.03333333333333333, "position_saturation_ratio_std": 0.05773502691896258, "mean_estimated_stiffness_n_per_m_mean": null, "mean_estimated_stiffness_n_per_m_std": null, "rise_time_s_mean": 0.3, "rise_time_s_std": 0.07071067811865474, "overshoot_ratio_mean": 0.15000000000000002, "overshoot_ratio_std": 0.07071067811865477, "settling_time_s_mean": 0.8, "settling_time_s_std": NaN},
    {"stiffness_estimator_method": "full", "task_name": "ramp_force_tracking", "object_material": "hard", "runs": 2, "passed_runs": 2, "contact_time_s_mean": 1.45, "contact_time_s_std": 0.07071067811865482, "tracking_start_time_s_mean": 1.85, "tracking_start_time_s_std": 0.07071067811865465, "rmse_n_mean": 0.31, "rmse_n_std": null, "mae_n_mean": 0.13, "mae_n_std": null, "peak_abs_error_n_mean": 0.6, "peak_abs_error_n_std": null, "mean_error_n_mean": 0.01, "mean_error_n_std": 0.0, "final_error_n_mean": 0.035, "final_error_n_std": 0.021213203435596427, "torque_saturation_ratio_mean": 0.15000000000000002, "torque_saturation_ratio_std": 0.07071067811865475, "position_saturation_ratio_mean": 0.025, "position_saturation_ratio_std": 0.035355339059327376, "mean_estimated_stiffness_n_per_m_mean": 500.0, "mean_estimated_stiffness_n_per_m_std": null, "rise_time_s_mean": 0.30000000000000004, "rise_time_s_std": 0.14142135623730953, "overshoot_ratio_mean": NaN, "overshoot_ratio_std": NaN, "settling_time_s_mean": 0.7, "settling_time_s_std": 0.28284271247461906},
    {"stiffness_estimator_method": "window-quadratic", "task_name": "step_force_tracking", "object_material": "medium", "runs": 1, "passed_runs": 0, "contact_time_s_mean": 0.75, "contact_time_s_std": null, "tracking_start_time_s_mean": 1.05, "tracking_start_time_s_std": null, "rmse_n_mean": 0.44, "rmse_n_std": null, "mae_n_mean": 0.2, "mae_n_std": null, "peak_abs_error_n_mean": 0.52, "peak_abs_error_n_std": null, "mean_error_n_mean": -0.02, "mean_error_n_std": null, "final_error_n_mean": -0.01, "final_error_n_std": null, "torque_saturation_ratio_mean": 0.0, "torque_saturation_ratio_std": null, "position_saturation_ratio_mean": 0.12, "position_saturation_ratio_std": null, "mean_estimated_stiffness_n_per_m_mean": 520.0, "mean_estimated_stiffness_n_per_m_std": null, "rise_time_s_mean": 0.3, "rise_time_s_std": NaN, "overshoot_ratio_mean": NaN, "overshoot_ratio_std": NaN, "settling_time_s_mean": 0.7, "settling_time_s_std": NaN}
        ]
        """
    ),
    "force_tracking_torque_adrc_tuning": (
        r"""
        [
    {"candidate_id": "zz-wide", "measurement_filter_cutoff_hz": 15.0, "controller_bandwidth_rad_s": 40.0, "observer_bandwidth_ratio": 8.0, "observer_bandwidth_rad_s": 320.0, "task_name": "step_force_tracking", "object_material": "medium", "runs": 2, "passed_runs": 1, "rmse_n_mean": 0.6, "rmse_n_std": 0.14142135623730948, "overshoot_ratio_mean": 0.25, "overshoot_ratio_std": null, "settling_time_s_mean": 1.1, "settling_time_s_std": null, "torque_saturation_ratio_mean": 0.01, "torque_saturation_ratio_std": 0.01414213562373095},
    {"candidate_id": "aa-base", "measurement_filter_cutoff_hz": 10.0, "controller_bandwidth_rad_s": 24.0, "observer_bandwidth_ratio": 10.0, "observer_bandwidth_rad_s": 240.0, "task_name": "step_force_tracking", "object_material": "medium", "runs": 2, "passed_runs": 1, "rmse_n_mean": 0.36, "rmse_n_std": 0.05656854249492381, "overshoot_ratio_mean": 0.12, "overshoot_ratio_std": null, "settling_time_s_mean": 1.05, "settling_time_s_std": 0.2121320343559642, "torque_saturation_ratio_mean": null, "torque_saturation_ratio_std": null},
    {"candidate_id": "zz-wide", "measurement_filter_cutoff_hz": 15.0, "controller_bandwidth_rad_s": 40.0, "observer_bandwidth_ratio": 8.0, "observer_bandwidth_rad_s": 320.0, "task_name": "ramp_force_tracking", "object_material": "medium", "runs": 1, "passed_runs": 1, "rmse_n_mean": 0.08, "rmse_n_std": null, "overshoot_ratio_mean": null, "overshoot_ratio_std": null, "settling_time_s_mean": null, "settling_time_s_std": null, "torque_saturation_ratio_mean": null, "torque_saturation_ratio_std": null}
        ]
        """
    ),
    "robotiq_discrete_force": (
        r"""
        [
    {"controller_variant": "fixed-step", "runs": 3, "passed_runs": 2, "safety_violations": 1, "safety_violation_duration_s_mean": 0.16666666666666666, "release_count_mean": 4.0, "action_count_mean": 47.0, "average_nonzero_action_step_mean": 0.011999999999999999, "command_movement_mean": 0.61, "reverse_count_mean": 1.3333333333333333, "oscillation_count_mean": 1.6666666666666667, "settling_time_s_mean": 0.49, "hold_ratio_mean": 0.8466666666666667, "steady_force_error_n_mean": 0.049999999999999996, "rmse_n_mean": 0.09999999999999999, "peak_overshoot_n_mean": 0.3, "prediction_mae_n_mean": 0.21},
    {"controller_variant": "adaptive-deadband", "runs": 2, "passed_runs": 1, "safety_violations": 1, "safety_violation_duration_s_mean": 0.75, "release_count_mean": 2.0, "action_count_mean": 38.5, "average_nonzero_action_step_mean": 0.014499999999999999, "command_movement_mean": 0.635, "reverse_count_mean": 0.5, "oscillation_count_mean": 3.5, "settling_time_s_mean": null, "hold_ratio_mean": 0.79, "steady_force_error_n_mean": 0.08, "rmse_n_mean": 0.125, "peak_overshoot_n_mean": 0.395, "prediction_mae_n_mean": null}
        ]
        """
    ),
    "friction_estimation_local_slip": (
        r"""
        [
    {"scenario": "positive_slip", "expect_local_slip": true, "runs": 3, "local_detections": 2, "detection_rate": 0.6666666666666666, "false_positive_rate": 0.0, "detection_lead_s_mean": 0.15, "detection_lead_s_std": 0.04242640687119285, "local_estimate_ratio_mean": 0.8500000000000001, "local_estimate_ratio_std": 0.07071067811865474, "passed_runs": 2, "control_qualified_runs": 1},
    {"scenario": "negative_noslip", "expect_local_slip": false, "runs": 2, "local_detections": 1, "detection_rate": 0.5, "false_positive_rate": 0.5, "detection_lead_s_mean": null, "detection_lead_s_std": null, "local_estimate_ratio_mean": 1.1, "local_estimate_ratio_std": null, "passed_runs": 1, "control_qualified_runs": 0},
    {"scenario": "single_probe", "expect_local_slip": true, "runs": 1, "local_detections": 1, "detection_rate": 1.0, "false_positive_rate": 0.0, "detection_lead_s_mean": 0.05, "detection_lead_s_std": null, "local_estimate_ratio_mean": null, "local_estimate_ratio_std": null, "passed_runs": 0, "control_qualified_runs": 1}
        ]
        """
    ),
}

_GOLDEN = {name: json.loads(text) for name, text in _GOLDEN_JSON.items()}

_AGGREGATE_FUNCTION = {
    "dm_admittance_tuning": "aggregate_rows",
    "force_tracking_ablation": "aggregate_rows",
    "force_tracking_controller_comparison": "aggregate_rows",
    "force_tracking_stiffness_estimator_comparison": "aggregate_rows",
    "force_tracking_torque_adrc_tuning": "_aggregate",
    "robotiq_discrete_force": "aggregate_rows",
    "friction_estimation_local_slip": "aggregate_rows",
}


def _assert_matches(actual: object, expected: object) -> None:
    """按黄金比较口径断言单个标量相等。

    整数、布尔、字符串与 None 精确相等；浮点允许 1e-12 相对容差，
    NaN 与 NaN 视为相等。
    """
    if isinstance(expected, bool) or expected is None or isinstance(expected, str):
        assert actual == expected
        return
    if isinstance(expected, int):
        assert actual == expected
        assert isinstance(actual, int)
        return
    if isinstance(expected, float):
        assert isinstance(actual, float)
        if math.isnan(expected):
            assert math.isnan(actual)
        elif math.isinf(expected):
            assert actual == expected
        else:
            assert actual == pytest.approx(expected, rel=1e-12, abs=1e-12)
        return
    raise AssertionError(f"未覆盖的黄金值类型：{expected!r}")


@pytest.mark.parametrize("stem", sorted(_AGGREGATE_FUNCTION))
def test_script_aggregation_matches_golden(stem: str) -> None:
    """各研究脚本的声明式聚合输出与手写实现的黄金基线一致。"""
    protocol = _protocol_module(stem)
    kwargs = {"minimum_ratio": 0.9} if stem == "dm_admittance_tuning" else {}
    actual = getattr(protocol, _AGGREGATE_FUNCTION[stem])(_SAMPLE_ROWS[stem], **kwargs)
    expected = _GOLDEN[stem]
    assert len(actual) == len(expected)
    for index, (got, want) in enumerate(zip(actual, expected, strict=True)):
        assert list(got) == list(want), f"行 {index} 列序不一致"
        for column in want:
            _assert_matches(got[column], want[column])
