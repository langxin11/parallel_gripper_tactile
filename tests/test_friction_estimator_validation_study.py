"""验证纯力局部起滑多种子 study 配置与聚合。"""

from pathlib import Path

import pytest

from parallel_gripper_tactile.studies.friction_estimator_validation import (
    load_friction_estimator_validation_config,
)
from parallel_gripper_tactile.studies.protocols import (
    friction_estimator_validation as protocol,
)

ROOT = Path(__file__).resolve().parents[1]


def test_default_friction_study_expands_scenarios_and_seeds() -> None:
    """默认 study 同时包含正例、负例和三个噪声种子。"""
    config = load_friction_estimator_validation_config(
        ROOT / "configs/research/friction_estimator_validation/study.yaml"
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

    protocol_rows = protocol.aggregate_rows(rows)
    aggregates = {row["scenario"]: row for row in protocol_rows}

    assert aggregates["positive"]["detection_rate"] == pytest.approx(0.5)
    assert aggregates["positive"]["false_positive_rate"] == 0.0
    assert aggregates["negative"]["false_positive_rate"] == 0.0
    assert aggregates["positive"]["control_qualified_runs"] == 1
