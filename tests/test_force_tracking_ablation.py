"""验证批量力控消融的选择解析与统计汇总。"""

from __future__ import annotations

import math

import pytest

from parallel_gripper_tactile.cli.experiments import (
    _aggregate_ablation_rows,
    _json_compatible,
    _parse_selection,
)


def _row(*, seed: int, rmse: float, stiffness: float) -> dict[str, object]:
    """创建一个包含全部聚合指标的最小结果行。"""
    return {
        "controller_variant": "pid-only",
        "object_material": "soft",
        "passed": True,
        "sensor_noise_seed": seed,
        "contact_time_s": 1.0,
        "tracking_start_time_s": 1.2,
        "rmse_n": rmse,
        "mae_n": 0.2,
        "peak_abs_error_n": 0.5,
        "mean_error_n": 0.1,
        "final_error_n": 0.05,
        "torque_saturation_ratio": 0.0,
        "position_saturation_ratio": 0.0,
        "mean_estimated_stiffness_n_per_m": stiffness,
    }


def test_ablation_selection_deduplicates_and_rejects_unknown_values() -> None:
    """逗号列表保留顺序、去重，并清楚报告未知档位。"""
    assert _parse_selection("hard, soft,hard", ("soft", "hard"), "materials") == (
        "hard",
        "soft",
    )
    with pytest.raises(ValueError, match="unknown values"):
        _parse_selection("soft,rubber", ("soft", "hard"), "materials")


def test_ablation_aggregation_skips_nan_and_json_uses_null() -> None:
    """PID-only 的无刚度估计值不会破坏均值或标准 JSON。"""
    rows = [
        _row(seed=1, rmse=0.2, stiffness=math.nan),
        _row(seed=2, rmse=0.4, stiffness=math.nan),
    ]

    aggregate = _aggregate_ablation_rows(rows)[0]

    assert aggregate["runs"] == 2
    assert aggregate["rmse_n_mean"] == pytest.approx(0.3)
    assert aggregate["rmse_n_std"] == pytest.approx(2**0.5 * 0.1)
    assert aggregate["mean_estimated_stiffness_n_per_m_mean"] is None
    compatible = _json_compatible({"runs": rows})
    assert isinstance(compatible, dict)
    assert compatible["runs"][0]["mean_estimated_stiffness_n_per_m"] is None  # type: ignore[index]
