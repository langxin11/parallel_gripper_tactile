"""配置重构前独立基线的不可变性与关键语义锚点。"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "tests/baselines/configuration_refactor_v1.json"
EXPECTED_SHA256 = "555e1d0945c5628b01cb6ed7ec494a0f04b3e047044d2b9728b81637fbaace96"


def _baseline() -> dict[str, object]:
    """读取已经冻结的迁移前快照。"""
    return json.loads(BASELINE.read_text(encoding="utf-8"))


def test_migration_baseline_is_immutable() -> None:
    """基线内容不能被新实现悄悄重写。"""
    assert hashlib.sha256(BASELINE.read_bytes()).hexdigest() == EXPECTED_SHA256
    baseline = _baseline()
    assert baseline["git_commit"] == "65201b5f711c9aafbdec28a14bc8a76db5a3d59f"
    assert len(baseline["yaml_sources"]) == 80


def test_migration_baseline_covers_profiles_and_tasks() -> None:
    """六个 profile 与十五个 task 均有完整领域值。"""
    baseline = _baseline()
    assert len(baseline["model_resources"]) == 7
    assert len(baseline["profiles"]) == 6
    tasks = baseline["tasks"]
    assert {family: len(entries) for family, entries in tasks.items()} == {
        "discrete_force": 1,
        "force_scheduling": 2,
        "force_tracking": 6,
        "friction_estimation": 6,
    }


def test_migration_baseline_preserves_admittance_differences() -> None:
    """导纳专用 MIT、接触、滤波和估计器差异均被基线捕获。"""
    profiles = _baseline()["profiles"]
    default = profiles["configs/dm_gripper.yaml"]["effective"]
    admittance = profiles["configs/dm_gripper_admittance.yaml"]["effective"]
    assert (default["control"]["mit"]["kp"], admittance["control"]["mit"]["kp"]) == (
        20.0,
        10.0,
    )
    assert (default["control"]["mit"]["kd"], admittance["control"]["mit"]["kd"]) == (
        0.63793536,
        5.0,
    )
    assert admittance["control"]["force"]["contact_threshold_n"] == 1.0
    assert admittance["control"]["force"]["filter_cutoff_hz"] == 2.0
    assert admittance["control"]["force"]["stiffness"]["enabled"] is False
    assert admittance["control"]["force"]["admittance"] is not None


def test_migration_baseline_preserves_study_matrices() -> None:
    """静态正式矩阵与诊断各 phase 的有序条件数保持可审计。"""
    studies = _baseline()["studies"]
    expected = {
        "force_tracking_controller_comparison": 162,
        "force_tracking_ablation": 36,
        "force_tracking_torque_adrc_tuning_coarse": 102,
        "friction_estimation_local_slip": 15,
        "force_tracking_stiffness_estimator_comparison": 81,
        "dm_admittance_tuning": 32,
        "robotiq_discrete_force": 60,
    }
    assert {name: studies[name]["condition_count"] for name in expected} == expected
    diagnosis = studies["force_tracking_diagnosis"]
    assert sum(phase["condition_count"] for phase in diagnosis.values()) == 39
    assert diagnosis["collision-geometry"]["condition_count"] == 5
