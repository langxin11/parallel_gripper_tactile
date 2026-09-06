"""验证纯力局部起滑多种子 study 配置与聚合。"""

from pathlib import Path
import importlib.util

import pytest

from parallel_gripper_tactile.studies.friction_estimation_local_slip import (
    load_local_slip_study_config,
)

ROOT = Path(__file__).resolve().parents[1]


def _protocol_module() -> object:
    """加载仓库内的局部起滑 study 入口脚本。"""
    path = ROOT / "scripts/experiments/friction_estimation_local_slip.py"
    spec = importlib.util.spec_from_file_location("friction_estimation_local_slip", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_default_local_slip_study_expands_scenarios_and_seeds() -> None:
    """默认 study 同时包含正例、负例和三个噪声种子。"""
    config = load_local_slip_study_config(
        ROOT / "configs/studies/friction_estimation_local_slip.yaml"
    )

    assert len(config.conditions()) == 15
    assert any(scenario.expect_local_slip for scenario in config.scenarios)
    assert any(not scenario.expect_local_slip for scenario in config.scenarios)
    assert all(scenario.task.is_absolute() for scenario in config.scenarios)


def test_aggregate_rows_reports_detection_and_false_positive_rates() -> None:
    """聚合分别报告正例检测率与负例误报率。"""
    rows = [
        {
            "scenario": "positive",
            "expect_local_slip": True,
            "local_slip_detected": True,
            "detection_lead_s": 1.0,
            "local_estimate_ratio": 0.8,
            "validation_passed": True,
            "control_candidate_qualified": True,
        },
        {
            "scenario": "positive",
            "expect_local_slip": True,
            "local_slip_detected": False,
            "detection_lead_s": None,
            "local_estimate_ratio": None,
            "validation_passed": False,
            "control_candidate_qualified": False,
        },
        {
            "scenario": "negative",
            "expect_local_slip": False,
            "local_slip_detected": False,
            "detection_lead_s": None,
            "local_estimate_ratio": None,
            "validation_passed": True,
            "control_candidate_qualified": False,
        },
    ]

    protocol = _protocol_module()
    aggregates = {row["scenario"]: row for row in protocol.aggregate_rows(rows)}

    assert aggregates["positive"]["detection_rate"] == pytest.approx(0.5)
    assert aggregates["positive"]["false_positive_rate"] == 0.0
    assert aggregates["negative"]["false_positive_rate"] == 0.0
    assert aggregates["positive"]["control_qualified_runs"] == 1
