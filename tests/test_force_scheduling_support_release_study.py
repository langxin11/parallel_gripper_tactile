"""验证自适应撤支撑研究矩阵、臂组合与生命周期产物。"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from parallel_gripper_tactile.studies.force_scheduling_support_release import (
    load_force_scheduling_support_release_config,
)
from parallel_gripper_tactile.research.study import compose_support_release_arm
from parallel_gripper_tactile.studies.protocols import (
    force_scheduling_support_release as protocol,
)


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "configs/research/adaptive_support_release_validation/study.yaml"


def _config():
    """加载正式研究矩阵。"""
    return load_force_scheduling_support_release_config(SOURCE)


def test_plan_registers_paired_arm_seed_matrix(tmp_path: Path) -> None:
    """计划按配置顺序展开六臂 × 三 seed，条件标识唯一。"""
    resolved = protocol.build_plan(_config(), arm_composer=compose_support_release_arm)

    ids = [condition.condition_id for condition in resolved.conditions]
    assert len(ids) == 18
    assert len(set(ids)) == 18
    assert ids[:3] == [
        "oracle-admittance-seed000",
        "oracle-admittance-seed001",
        "oracle-admittance-seed002",
    ]
    arms = {condition.parameters["arm"] for condition in resolved.conditions}
    assert arms == {
        "oracle-admittance",
        "oracle-pid",
        "adaptive-admittance",
        "adaptive-pid",
        "adaptive-admittance-multirate",
        "adaptive-admittance-risk",
    }


def test_every_arm_composes_with_declared_controller_and_scheduler() -> None:
    """每个对照臂都能完成组合预检并声明预期控制器与调度器。"""
    config = _config()
    composed = {
        arm.name: compose_support_release_arm(arm, seed=config.seeds.values()[0])
        for arm in config.arms
    }

    assert composed["oracle-admittance"].selection.scheduler.name == "oracle"
    assert composed["oracle-admittance"].selection.controller.name == "admittance"
    assert composed["oracle-pid"].selection.controller.name == "pid-torque-ff"
    assert composed["adaptive-pid"].selection.controller.name == "pid-torque-ff"
    for name in (
        "adaptive-admittance",
        "adaptive-pid",
        "adaptive-admittance-multirate",
        "adaptive-admittance-risk",
    ):
        assert composed[name].selection.scheduler.name == "adaptive"
    # 多速率臂的任务带独立触觉采样，风险臂启用风险与摩擦更新。
    assert composed["adaptive-admittance-multirate"].task.tactile_sampling is not None
    assert composed["adaptive-admittance-risk"].scheduler.risk_enabled
    assert composed["adaptive-admittance-risk"].scheduler.friction_update_enabled
    # oracle 与 adaptive 使用相同力限与速率，保证信息来源是唯一系统差异。
    assert composed["oracle-admittance"].scheduler.min_force_n == pytest.approx(1.0)
    assert composed["adaptive-admittance"].scheduler.load.min_force_n == pytest.approx(1.0)
    assert composed["oracle-admittance"].scheduler.max_force_rate_n_s == pytest.approx(50.0)
    assert composed["adaptive-admittance"].scheduler.load.max_force_rate_n_s == pytest.approx(50.0)


def test_run_study_writes_manifest_and_arm_aggregates(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """执行层按计划逐臂派发并写出臂级聚合与汇总图。"""
    config = _config()

    def fake_execute(**kwargs: object):
        run_path = Path(str(kwargs["output_root"])) / f"{kwargs['run_prefix']}-synthetic"
        run_path.mkdir(parents=True)
        return SimpleNamespace(path=run_path), protocol_stub_result()

    def protocol_stub_result():
        from parallel_gripper_tactile.experiments.force_scheduling import ForceSchedulingResult

        return ForceSchedulingResult(
            contact_time_s=1.0,
            scenario_start_time_s=4.0,
            scenario_duration_s=4.0,
            max_tangential_displacement_m=0.001,
            force_tracking_rmse_n=0.2,
            mean_target_force_n=3.0,
            peak_target_force_n=4.0,
            final_target_force_n=3.5,
            minimum_friction_margin_n=0.4,
            peak_friction_utilization=0.6,
            target_force_maximum_ratio=0.9,
            target_force_rate_limited_ratio=0.0,
            simulation_stable=True,
            slip_passed=True,
            force_tracking_passed=True,
        )

    monkeypatch.setattr(protocol, "execute_force_scheduling", fake_execute)
    result = protocol.run_study(
        config,
        arm_composer=compose_support_release_arm,
        config_source=SOURCE,
        study_directory=tmp_path,
    )

    manifest = json.loads((result / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == "completed"
    assert manifest["planned_condition_count"] == 18
    assert manifest["all_conditions_passed"] is True
    summary = json.loads((result / "summary.json").read_text(encoding="utf-8"))
    assert len(summary["runs"]) == 18
    arms = {row["arm"] for row in summary["aggregates"]}
    assert len(arms) == 6
    assert (result / "support_release_validation.png").exists()
