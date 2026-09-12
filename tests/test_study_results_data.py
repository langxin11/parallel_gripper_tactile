"""研究级报告数据块生成脚本的验证。"""

from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
RUN_RELATIVE = "runs/dm_gripper/force-track/condition-000"


def _module() -> object:
    """加载报告数据脚本而不执行命令行入口。"""
    path = ROOT / "scripts" / "reports" / "study_results_data.py"
    spec = importlib.util.spec_from_file_location("study_results_data_script", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    """写入带表头的 CSV 产物。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _summary_row(**overrides: object) -> dict[str, object]:
    """构造一条带 run_directory 的 summary 记录。"""
    row: dict[str, object] = {"run_directory": RUN_RELATIVE}
    row.update(overrides)
    return row


def _write_manifest(root: Path, study_path: str) -> None:
    """为 study 写入首个条件的 manifest.json。"""
    run_dir = root / study_path / RUN_RELATIVE
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "manifest.json").write_text(
        json.dumps({"git": {"commit": "abcdef1234567890", "dirty": False}}),
        encoding="utf-8",
    )


def _fixture_tree(tmp_path: Path) -> Path:
    """造一棵最小 study 产物树，列名与真实产物一致。"""
    module = _module()
    paths: dict[str, str] = dict(module.STUDIES)

    tasks = ("step_force_tracking", "ramp_force_tracking", "mixed_waypoint_force_tracking")
    variants = {
        "full": 0.20,
        "pid-torque-ff": 0.22,
        "pid-stiffness-ff": 0.30,
        "pid-only": 0.32,
        "adrc-torque": 0.40,
        "pid-stiffness-limit": 0.80,
    }
    controller_dir = tmp_path / paths["controller"]
    _write_csv(
        controller_dir / "summary.csv",
        [
            _summary_row(
                controller_variant=variant,
                task_name=task,
                object_material="hard",
                sensor_noise_seed=0,
                passed="True",
                rmse_n=rmse,
                mae_n=rmse / 2,
                overshoot_ratio=0.01,
                torque_saturation_ratio=0.0,
                position_saturation_ratio=0.0,
            )
            for variant, rmse in variants.items()
            for task in tasks
        ],
    )

    ablation_dir = tmp_path / paths["ablation"]
    _write_csv(
        ablation_dir / "summary.csv",
        [
            _summary_row(
                controller_variant=variant,
                object_material=material,
                rmse_n=rmse,
                passed="True",
            )
            for variant, rmse in (
                ("full", 0.10),
                ("pid-torque-ff", 0.11),
                ("pid-stiffness-ff", 0.30),
                ("pid-only", 0.31),
            )
            for material in ("medium", "hard", "stiff")
        ],
    )

    coarse_dir = tmp_path / paths["adrc_coarse"]
    _write_csv(
        coarse_dir / "summary.csv",
        [_summary_row(task_name=task, rmse_n=0.05, passed="True") for task in tasks],
    )
    _write_csv(
        coarse_dir / "candidate_ranking.csv",
        [
            {
                "rank": "1",
                "candidate_id": "fc40-wc60-ratio4",
                "measurement_filter_cutoff_hz": "40.0",
                "controller_bandwidth_rad_s": "60.0",
                "observer_bandwidth_ratio": "4.0",
                "feasible": "true",
                "step_overshoot_ratio_mean": "0.20",
                "ramp_rmse_ratio_to_baseline": "1.0",
                "mixed_rmse_ratio_to_baseline": "1.0",
                "max_torque_saturation_ratio": "0.0",
            },
            {
                "rank": "2",
                "candidate_id": "fc30-wc30-ratio2",
                "measurement_filter_cutoff_hz": "30.0",
                "controller_bandwidth_rad_s": "30.0",
                "observer_bandwidth_ratio": "2.0",
                "feasible": "false",
                "step_overshoot_ratio_mean": "0.05",
                "ramp_rmse_ratio_to_baseline": "3.0",
                "mixed_rmse_ratio_to_baseline": "4.0",
                "max_torque_saturation_ratio": "0.0",
            },
        ],
    )

    confirm_dir = tmp_path / paths["adrc_confirm"]
    _write_csv(
        confirm_dir / "summary.csv",
        [
            _summary_row(task_name=task, rmse_n=0.05, passed="True", torque_saturation_ratio=0.0)
            for task in tasks
        ],
    )
    _write_csv(
        confirm_dir / "aggregate.csv",
        [
            {
                "task_name": "step_force_tracking",
                "object_material": "hard",
                "rmse_n_mean": "0.60",
                "rmse_n_std": "0.01",
                "overshoot_ratio_mean": "0.20",
                "torque_saturation_ratio_mean": "0.0",
            },
            {
                "task_name": "ramp_force_tracking",
                "object_material": "hard",
                "rmse_n_mean": "0.04",
                "rmse_n_std": "0.001",
                "overshoot_ratio_mean": "",
                "torque_saturation_ratio_mean": "0.0",
            },
        ],
    )
    _write_csv(
        confirm_dir / "candidate_ranking.csv",
        [{"candidate_id": "fc40-wc60-ratio4", "step_overshoot_ratio_mean": "0.20"}],
    )

    estimator_dir = tmp_path / paths["estimator"]
    _write_csv(
        estimator_dir / "summary.csv",
        [
            _summary_row(
                stiffness_estimator_method=estimator,
                task_name=task,
                rmse_n=rmse,
                mean_estimated_stiffness_n_per_m=stiffness,
                passed="True",
            )
            for estimator, rmse, stiffness in (
                ("secant_ewma", 0.20, 2500.0),
                ("window_linear", 0.21, 2000.0),
                ("window_quadratic", 0.205, 3500.0),
            )
            for task in tasks
        ],
    )

    admittance_dir = tmp_path / paths["admittance"]
    _write_csv(
        admittance_dir / "summary.csv",
        [_summary_row(task_name="dm_admittance_ramp", rmse_n=0.01) for _ in range(2)],
    )
    _write_csv(
        admittance_dir / "candidate_ranking.csv",
        [
            {
                "rank": "1",
                "mass_kg": "0.2",
                "damping_ns_m": "15.0",
                "stiffness_n_m": "1.0",
                "filter_cutoff_hz": "2.0",
                "velocity_limit_rad_s": "0.05",
                "approach_feedforward_force_n": "0.5",
                "rmse_n_mean": "0.007",
                "stable": "true",
            },
            {
                "rank": "2",
                "mass_kg": "0.2",
                "damping_ns_m": "15.0",
                "stiffness_n_m": "1.0",
                "filter_cutoff_hz": "2.0",
                "velocity_limit_rad_s": "0.1",
                "approach_feedforward_force_n": "1.0",
                "rmse_n_mean": "0.02",
                "stable": "true",
            },
        ],
    )

    robotiq_dir = tmp_path / paths["robotiq"]
    _write_csv(
        robotiq_dir / "summary.csv",
        [
            _summary_row(controller_variant=variant, passed="True")
            for variant in ("quantized-pi", "fixed-step", "dynamic-step")
        ],
    )
    _write_csv(
        robotiq_dir / "aggregate.csv",
        [
            {
                "controller_variant": variant,
                "passed_runs": passed,
                "rmse_n_mean": rmse,
                "hold_ratio_mean": hold,
                "steady_force_error_n_mean": steady,
                "action_count_mean": actions,
                "reverse_count_mean": reverse,
            }
            for variant, passed, rmse, hold, steady, actions, reverse in (
                ("quantized-pi", 0, 0.20, 0.65, 0.085, 166.0, 92.0),
                ("fixed-step", 2, 0.33, 0.39, 0.124, 100.0, 31.0),
                ("dynamic-step", 12, 0.50, 0.84, 0.234, 65.0, 3.5),
            )
        ],
    )

    friction_dir = tmp_path / paths["friction"]
    _write_csv(
        friction_dir / "summary.csv",
        [
            _summary_row(
                scenario=scenario,
                expect_local_slip=expect,
                local_slip_detected=detected,
                validation_passed=validation,
                control_candidate_qualified=qualified,
            )
            for scenario, expect, detected, validation, qualified in (
                ("low_friction_probe", "True", "True", "True", "True"),
                ("no_slip_low_probe", "False", "False", "True", "False"),
            )
        ],
    )
    _write_csv(
        friction_dir / "aggregate.csv",
        [
            {
                "scenario": "low_friction_probe",
                "expect_local_slip": "True",
                "detection_rate": "1.0",
                "false_positive_rate": "0.0",
                "detection_lead_s_mean": "0.15",
                "passed_runs": "1",
            },
            {
                "scenario": "no_slip_low_probe",
                "expect_local_slip": "False",
                "detection_rate": "0.0",
                "false_positive_rate": "0.0",
                "detection_lead_s_mean": "",
                "passed_runs": "3",
            },
        ],
    )

    for study_path in paths.values():
        _write_manifest(tmp_path, study_path)
    return tmp_path


def test_build_block_emits_literal_data_only(tmp_path: Path) -> None:
    """数据块由产物算出的字面量组成，不包含编译期读取产物的调用。"""
    module = _module()
    root = _fixture_tree(tmp_path)

    block = module.build_block(root)

    assert "#let source-rows = (" not in block
    assert "#let controller-rows = (" in block
    assert '"pid-stiffness-limit"' in block
    assert "conditions: 18" in block
    assert "feasible: true" in block
    assert "overshoot: none" in block
    assert "csv(" not in block
    assert "json(" not in block


def test_updated_report_replaces_only_data_block(tmp_path: Path) -> None:
    """刷新数据块时保留块外正文。"""
    module = _module()
    root = _fixture_tree(tmp_path)
    report = tmp_path / "study_results.typ"
    report.write_text(
        '#import "x": y\n'
        + module.BEGIN_MARKER
        + "\n// 旧数据\n"
        + module.END_MARKER
        + "\n= 正文\n",
        encoding="utf-8",
    )

    updated = module.updated_report(root, report)

    assert updated.startswith('#import "x": y\n')
    assert updated.endswith("= 正文\n")
    assert "// 旧数据" not in updated
    assert "#let friction-rows = (" in updated


def test_updated_report_requires_markers(tmp_path: Path) -> None:
    """缺少标记行时给出可诊断的错误。"""
    module = _module()
    report = tmp_path / "broken.typ"
    report.write_text("= 没有数据块\n", encoding="utf-8")

    with pytest.raises(module.ReportDataError):
        module.updated_report(tmp_path, report)


def test_check_mode_detects_stale_block(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """``--check`` 在数据块过期时返回 1，刷新后返回 0。"""
    module = _module()
    root = _fixture_tree(tmp_path)
    report = tmp_path / "study_results.typ"
    report.write_text(module.BEGIN_MARKER + "\n// 旧数据\n" + module.END_MARKER, encoding="utf-8")

    assert module.main(["--check", "--root", str(root), "--report", str(report)]) == 1
    assert module.main(["--root", str(root), "--report", str(report)]) == 0
    assert module.main(["--check", "--root", str(root), "--report", str(report)]) == 0
    assert "数据块与产物一致" in capsys.readouterr().out


def test_committed_report_block_matches_artifacts() -> None:
    """仓库中的报告数据块与本地产物保持一致；产物缺失时跳过。"""
    module = _module()
    missing = [path for path in module.STUDIES.values() if not (ROOT / path).is_dir()]
    if missing:
        pytest.skip(f"本地缺少 study 产物，跳过数据块一致性校验：{missing[0]}")

    assert module.main(["--check", "--root", str(ROOT)]) == 0
