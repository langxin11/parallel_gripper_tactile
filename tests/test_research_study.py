"""验证正式 Hydra study 的唯一矩阵、计划和 Multirun 防护。"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace

from hydra import compose, initialize_config_dir
import pytest

from parallel_gripper_tactile.research import REPOSITORY_ROOT
from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingResult
from parallel_gripper_tactile.research.hydra_support import register_resolvers, resolved_mapping
from parallel_gripper_tactile.research.study import (
    ResearchStudySetupError,
    execute_research_study,
    resolve_research_study,
)
from parallel_gripper_tactile.studies.force_tracking_ablation import load_study_config
from parallel_gripper_tactile.studies.force_tracking_comparison import load_comparison_config
from parallel_gripper_tactile.studies.force_tracking_torque_adrc_tuning import (
    ForceTrackingTorqueAdrcTuningConfig,
    TorqueAdrcCandidate,
    load_torque_adrc_tuning_config,
)
from parallel_gripper_tactile.studies.lifecycle import file_sha256
from parallel_gripper_tactile.studies.protocols import (
    force_tracking_torque_adrc_tuning as torque_protocol,
)
from parallel_gripper_tactile.studies.tabular import write_rows_csv


CONFIG_ROOT = REPOSITORY_ROOT / "configs" / "research"


def _resolved(name: str, *, overrides: list[str] | None = None):
    """组合并解析一个正式研究 preset。"""
    register_resolvers()
    with initialize_config_dir(version_base="1.3", config_dir=str(CONFIG_ROOT)):
        config = compose(config_name=name, overrides=overrides or [])
    return resolve_research_study(resolved_mapping(config))


def _write_coarse_reference(
    directory: Path,
    config: ForceTrackingTorqueAdrcTuningConfig,
) -> tuple[TorqueAdrcCandidate, ...]:
    """生成 schema、候选全集和摘要均有效的 coarse 谱系 fixture。"""
    directory.mkdir(parents=True, exist_ok=True)
    candidates = config.stage_candidates("coarse")
    rows = [
        {
            "rank": rank,
            "candidate_id": candidate.identifier,
            "measurement_filter_cutoff_hz": candidate.measurement_filter_cutoff_hz,
            "controller_bandwidth_rad_s": candidate.controller_bandwidth_rad_s,
            "observer_bandwidth_ratio": candidate.observer_bandwidth_ratio,
            "observer_bandwidth_rad_s": candidate.observer_bandwidth_rad_s,
            "feasible": "true",
            "step_overshoot_ratio_mean": 0.01 * rank,
            "step_rmse_n_mean": 0.1 * rank,
            "ramp_rmse_n_mean": 0.1 * rank,
            "mixed_rmse_n_mean": 0.1 * rank,
            "ramp_rmse_ratio_to_baseline": 1.0,
            "mixed_rmse_ratio_to_baseline": 1.0,
            "max_torque_saturation_ratio": 0.0,
        }
        for rank, candidate in enumerate(candidates, start=1)
    ]
    ranking = write_rows_csv(directory / "candidate_ranking.csv", rows)
    coarse_plan = torque_protocol.build_plan(config, stage="coarse")
    manifest = {
        "lifecycle_schema_version": 1,
        "study_kind": "force_tracking_torque_adrc_tuning",
        "stage": "coarse",
        "state": "completed",
        "study_definition_sha256": coarse_plan.study_definition_sha256,
        "scientific_configuration_sha256": coarse_plan.scientific_configuration_sha256,
        "execution_error_count": 0,
        "lifecycle_failures": [],
        "artifacts": [ranking.name],
        "artifact_sha256": {ranking.name: file_sha256(ranking)},
    }
    (directory / "study_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return candidates


def _entry_module():
    """加载 study 薄入口以测试调度前防护。"""
    path = REPOSITORY_ROOT / "scripts/research/study.py"
    spec = importlib.util.spec_from_file_location("research_study_entry", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_formal_comparison_preserves_the_complete_ordered_matrix() -> None:
    """迁移后的 162 条条件与旧 Pydantic study 的完整 tuple 逐项一致。"""
    resolved = _resolved("force_tracking_controller_comparison")
    legacy = load_comparison_config(
        REPOSITORY_ROOT / "configs/studies/force_tracking_controller_comparison.yaml"
    )
    expected_controllers = (
        "pid-only",
        "pid-torque-ff",
        "pid-stiffness-ff",
        "full",
        "pid-stiffness-limit",
        "adrc-torque",
    )

    actual = tuple(
        (row["controller"], Path(str(row["task"])), row["material"], row["seed"])
        for row in resolved.conditions
    )
    assert legacy.controllers == expected_controllers
    assert actual == legacy.conditions()
    assert len(set(str(row["condition_id"]) for row in resolved.conditions)) == 162


def test_formal_ablation_preserves_pairing_and_complete_matrix() -> None:
    """迁移后的 36 条 PID 2×2 条件保持材料—seed 配对关系。"""
    resolved = _resolved("force_tracking_ablation")
    legacy = load_study_config(REPOSITORY_ROOT / "configs/studies/force_tracking_ablation.yaml")

    actual = tuple((row["controller"], row["material"], row["seed"]) for row in resolved.conditions)
    assert actual == legacy.conditions()
    assert len({row["pair_key"] for row in resolved.conditions}) == 9


def test_formal_local_slip_preserves_scenario_and_seed_matrix() -> None:
    """迁移后的 15 条局部起滑条件与旧 study 配置逐项一致。"""
    from parallel_gripper_tactile.studies.friction_estimation_local_slip import (
        load_local_slip_study_config,
    )

    resolved = _resolved("friction_estimation_local_slip")
    legacy = load_local_slip_study_config(
        REPOSITORY_ROOT / "configs/studies/friction_estimation_local_slip.yaml"
    )

    actual = tuple(
        (
            Path(str(row["task_path"])),
            row["expect_local_slip"],
            row["seed"],
        )
        for row in resolved.conditions
    )
    expected = tuple(
        (scenario.task, scenario.expect_local_slip, seed) for scenario, seed in legacy.conditions()
    )
    assert actual == expected
    assert len({row["condition_id"] for row in resolved.conditions}) == 15


def test_local_slip_plan_registers_expected_condition_count(tmp_path: Path) -> None:
    """局部起滑 plan 产物登记 15 条条件且不创建任何 run。"""
    resolved = _resolved("friction_estimation_local_slip")
    result = execute_research_study(
        resolved,
        hydra_output_directory=tmp_path,
        provenance={"choices": {}, "overrides": []},
    )

    plan = json.loads((result / "plan.json").read_text(encoding="utf-8"))
    assert plan["study"] == "friction_estimation_local_slip"
    assert plan["condition_count"] == 15
    manifest = json.loads((result / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == "planned"
    assert not (result / "runs").exists()


def test_local_slip_protocol_preserves_validation_semantics(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """局部起滑以 validation_passed 作为科学验收并保留期望汇总字段。"""
    from parallel_gripper_tactile.experiments.friction_estimation import (
        FrictionEstimationResult,
        FrictionEstimationTask,
    )
    from parallel_gripper_tactile.studies.friction_estimation_local_slip import (
        load_local_slip_study_config,
    )
    from parallel_gripper_tactile.studies.protocols import (
        friction_estimation_local_slip as protocol,
    )

    source = REPOSITORY_ROOT / "configs/studies/friction_estimation_local_slip.yaml"
    original = load_local_slip_study_config(source)
    scenario = next(s for s in original.scenarios if s.expect_local_slip)
    config = original.model_copy(
        update={
            "scenarios": (scenario,),
            "seeds": original.seeds.model_copy(update={"count": 1}),
        }
    )

    def fake_execute(**kwargs: object):
        run_path = Path(str(kwargs["output_root"])) / f"{kwargs['run_prefix']}-synthetic"
        run_path.mkdir(parents=True)
        result = FrictionEstimationResult(
            true_friction_coefficient=0.4,
            estimated_friction_coefficient=0.35,
            raw_friction_coefficient=0.36,
            estimate_ratio=0.875,
            absolute_estimation_error=0.05,
            slip_detected=True,
            using_fallback=False,
            probe_detection_time_s=2.5,
            probe_force_at_detection_n=1.0,
            max_probe_displacement_m=0.001,
            max_hold_displacement_m=0.002,
            hold_force_tracking_rmse_n=0.05,
            minimum_hold_friction_margin_n=0.5,
            peak_active_taxel_count=4,
            peak_local_friction_ratio=0.9,
            local_weighted_ratio_at_detection=0.85,
            local_ratio_p90_at_detection=0.9,
            local_slip_detection_time_s=2.0,
            local_slip_detected_taxel_count=2,
            local_left_friction_estimate=None,
            local_right_friction_estimate=None,
            simulation_stable=True,
            detection_passed=True,
            conservatism_passed=True,
            informativeness_passed=True,
            probe_slip_passed=True,
            hold_slip_passed=True,
            force_tracking_passed=True,
            detection_reason="synthetic",
            detection_features={"peak_local_friction_ratio": 0.9},
            relative_estimation_error=0.125,
            probe_displacement_limit_exceeded=False,
            hold_succeeded=True,
        )
        return SimpleNamespace(path=run_path), result

    monkeypatch.setattr(protocol, "execute_friction_estimation", fake_execute)
    protocol.run_study(config, config_source=source, study_directory=tmp_path)

    manifest = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == "completed"
    assert manifest["scientific_failure_count"] == 1
    assert manifest["all_event_expectations_passed"] is True
    assert manifest["all_expectations_passed"] is False
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    row = summary["runs"][0]
    task_name = FrictionEstimationTask.load(scenario.task).name
    assert row["scenario"] == task_name
    assert row["detection_lead_s"] == pytest.approx(0.5)
    assert row["validation_passed"] is False
    assert row["control_candidate_qualified"] is False
    assert (tmp_path / "local_slip_validation.png").exists()
    assert (tmp_path / "local_slip_validation.pdf").exists()


def test_local_slip_protocol_records_condition_exceptions(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """单条件异常仍生成完整失败账本，不伪装成未执行计划。"""
    from parallel_gripper_tactile.experiments.friction_estimation import (
        FrictionEstimationTask,
    )
    from parallel_gripper_tactile.studies.friction_estimation_local_slip import (
        load_local_slip_study_config,
    )
    from parallel_gripper_tactile.studies.protocols import (
        friction_estimation_local_slip as protocol,
    )

    source = REPOSITORY_ROOT / "configs/studies/friction_estimation_local_slip.yaml"
    original = load_local_slip_study_config(source)
    config = original.model_copy(
        update={
            "scenarios": original.scenarios[:1],
            "seeds": original.seeds.model_copy(update={"count": 1}),
        }
    )

    def fail(**kwargs: object):
        raise RuntimeError("synthetic failure")

    monkeypatch.setattr(protocol, "execute_friction_estimation", fail)
    result = protocol.run_study(config, config_source=source, study_directory=tmp_path)

    task_name = FrictionEstimationTask.load(config.scenarios[0].task).name
    manifest = json.loads((result / "study_manifest.json").read_text(encoding="utf-8"))
    failures = json.loads((result / "failed_conditions.json").read_text(encoding="utf-8"))
    assert manifest["state"] == "failed"
    assert manifest["planned_condition_count"] == 1
    assert failures[0]["condition_id"] == f"{task_name}-seed000"
    assert failures[0]["error_type"] == "RuntimeError"


def test_plan_serializes_the_same_validated_condition_collection(tmp_path: Path) -> None:
    """计划产物直接序列化 resolver 交给执行层的同一条件集合。"""
    resolved = _resolved("force_tracking_ablation")
    result = execute_research_study(
        resolved,
        hydra_output_directory=tmp_path,
        provenance={"choices": {}, "overrides": []},
    )

    plan = json.loads((result / "plan.json").read_text(encoding="utf-8"))
    assert plan["artifact_kind"] == "study_plan"
    assert plan["validated"] is True
    assert plan["conditions"] == list(resolved.conditions)
    manifest = json.loads((result / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == "planned"
    assert manifest["scientific_configuration_sha256"] == (
        resolved.plan.scientific_configuration_sha256
    )
    assert manifest["provenance"] == {"choices": {}, "overrides": []}
    assert manifest["preflight"]["status"] == "passed"
    assert set(manifest["artifacts"]) == {
        "effective_study_configuration.json",
        "composition_provenance.json",
        "plan.json",
    }
    assert not (result / "runs").exists()


def test_formal_torque_coarse_preserves_all_102_ordered_conditions() -> None:
    """正式 coarse 的 34 候选×3 任务完整顺序与领域模型逐项一致。"""
    resolved = _resolved("force_tracking_torque_adrc_tuning_coarse")
    config = load_torque_adrc_tuning_config(
        REPOSITORY_ROOT / "configs/studies/force_tracking_torque_adrc_tuning.yaml"
    )
    actual = tuple(
        (
            TorqueAdrcCandidate(
                float(row["measurement_filter_cutoff_hz"]),
                float(row["controller_bandwidth_rad_s"]),
                float(row["observer_bandwidth_ratio"]),
            ),
            Path(str(row["task_path"])),
            row["object_material"],
            row["sensor_noise_seed"],
        )
        for row in resolved.conditions
    )
    assert actual == config.conditions("coarse")
    assert len(config.stage_candidates("coarse")) == 34
    assert len(actual) == 102


def test_torque_confirm_uses_validated_ranking_and_preserves_order(tmp_path: Path) -> None:
    """confirm 只按 coarse 可行排名选前五，并在缺席时追加基线。"""
    config = load_torque_adrc_tuning_config(
        REPOSITORY_ROOT / "configs/studies/force_tracking_torque_adrc_tuning.yaml"
    )
    ranked = _write_coarse_reference(tmp_path, config)
    resolved = _resolved(
        "force_tracking_torque_adrc_tuning_confirm",
        overrides=[f"study.coarse_study_dir={tmp_path}"],
    )
    selected = tuple(dict.fromkeys((*ranked[: config.confirm_top_candidates], config.baseline)))
    actual = tuple(
        (
            str(row["candidate_id"]),
            Path(str(row["task_path"])),
            row["object_material"],
            row["sensor_noise_seed"],
        )
        for row in resolved.conditions
    )
    expected = tuple(
        (candidate.identifier, task, material, seed)
        for candidate, task, material, seed in config.conditions("confirm", candidates=selected)
    )
    assert actual == expected
    assert len(selected) == 6
    assert len(actual) == 162


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("stage", "confirm"),
        ("study_kind", "force_tracking_ablation"),
        ("study_definition_sha256", "0" * 64),
        ("scientific_configuration_sha256", None),
    ],
)
def test_torque_confirm_rejects_incompatible_coarse_manifest(
    tmp_path: Path, field: str, value: object
) -> None:
    """confirm 在形成任何计划前拒绝错误类型、阶段或配置哈希。"""
    config = load_torque_adrc_tuning_config(
        REPOSITORY_ROOT / "configs/studies/force_tracking_torque_adrc_tuning.yaml"
    )
    _write_coarse_reference(tmp_path, config)
    manifest_path = tmp_path / "study_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest[field] = value
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError):
        torque_protocol.build_plan(config, stage="confirm", coarse_study_dir=tmp_path)


def test_torque_confirm_rejects_missing_or_modified_ranking(tmp_path: Path) -> None:
    """confirm 校验排名产物的登记、存在性和内容摘要。"""
    config = load_torque_adrc_tuning_config(
        REPOSITORY_ROOT / "configs/studies/force_tracking_torque_adrc_tuning.yaml"
    )
    _write_coarse_reference(tmp_path, config)
    (tmp_path / "candidate_ranking.csv").write_text("modified\n", encoding="utf-8")
    with pytest.raises(ValueError, match="digest mismatch"):
        torque_protocol.build_plan(config, stage="confirm", coarse_study_dir=tmp_path)


def test_confirm_reports_configuration_and_preflight_failures_separately(
    tmp_path: Path,
) -> None:
    """缺少 coarse 路径属于配置失败，不兼容 coarse manifest 属于预检失败。"""
    with pytest.raises(ResearchStudySetupError) as missing:
        _resolved("force_tracking_torque_adrc_tuning_confirm")
    assert missing.value.stage == "configuration"

    config = load_torque_adrc_tuning_config(
        REPOSITORY_ROOT / "configs/studies/force_tracking_torque_adrc_tuning.yaml"
    )
    _write_coarse_reference(tmp_path, config)
    manifest_path = tmp_path / "study_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["stage"] = "confirm"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ResearchStudySetupError) as incompatible:
        _resolved(
            "force_tracking_torque_adrc_tuning_confirm",
            overrides=[f"study.coarse_study_dir={tmp_path}"],
        )
    assert incompatible.value.stage == "preflight"


def test_study_hash_ignores_output_root_but_changes_with_science() -> None:
    """输出位置不进入科学哈希，而验收约束变化必须改变哈希。"""
    config = load_torque_adrc_tuning_config(
        REPOSITORY_ROOT / "configs/studies/force_tracking_torque_adrc_tuning.yaml"
    )
    original = torque_protocol.build_plan(config, stage="coarse")
    moved = torque_protocol.build_plan(
        config.model_copy(update={"output_root": Path("/tmp/unrelated-output")}),
        stage="coarse",
    )
    changed = torque_protocol.build_plan(
        config.model_copy(
            update={
                "constraints": config.constraints.model_copy(
                    update={"max_torque_saturation_ratio": 0.02}
                )
            }
        ),
        stage="coarse",
    )
    assert moved.scientific_configuration_sha256 == original.scientific_configuration_sha256
    assert changed.scientific_configuration_sha256 != original.scientific_configuration_sha256


def test_study_hash_is_independent_of_current_working_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """同一已解析科学配置从不同 cwd 生成相同哈希。"""
    from parallel_gripper_tactile.studies.protocols import force_tracking_ablation as protocol

    config = load_study_config(REPOSITORY_ROOT / "configs/studies/force_tracking_ablation.yaml")
    expected = protocol.build_plan(config).scientific_configuration_sha256
    monkeypatch.chdir(tmp_path)
    actual = protocol.build_plan(config).scientific_configuration_sha256
    assert actual == expected


def test_execution_passes_the_exact_resolved_plan_to_protocol(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """执行层不重新展开条件，直接传递解析阶段已校验的计划实例。"""
    from parallel_gripper_tactile.studies.protocols import force_tracking_ablation as protocol

    resolved = _resolved("force_tracking_ablation")
    run_selection = resolved.selection.model_copy(
        update={"execution": resolved.selection.execution.model_copy(update={"mode": "run"})}
    )
    runnable = replace(resolved, selection=run_selection)
    captured: dict[str, object] = {}

    def fake_run(config: object, **kwargs: object) -> Path:
        captured.update(kwargs)
        return Path(str(kwargs["study_directory"]))

    monkeypatch.setattr(protocol, "run_study", fake_run)
    execute_research_study(
        runnable,
        hydra_output_directory=tmp_path,
        provenance={"choices": {}, "overrides": []},
    )
    assert captured["study_plan"] is resolved.plan
    assert {Path(path).name for path in captured["additional_artifacts"]} == {
        "effective_study_configuration.json",
        "composition_provenance.json",
    }


def test_protocol_records_condition_exceptions_and_finishes_manifest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """单条件异常仍生成完整失败账本，不伪装成未执行计划。"""
    from parallel_gripper_tactile.studies.protocols import force_tracking_ablation as protocol

    config = load_study_config(
        REPOSITORY_ROOT / "configs/studies/smoke/force_tracking_ablation.yaml"
    )

    def fail(**kwargs: object):
        raise RuntimeError("synthetic failure")

    monkeypatch.setattr(protocol, "execute_force_tracking", fail)
    result = protocol.run_study(
        config,
        config_source=REPOSITORY_ROOT / "configs/studies/smoke/force_tracking_ablation.yaml",
        study_directory=tmp_path,
    )

    manifest = json.loads((result / "study_manifest.json").read_text(encoding="utf-8"))
    failures = json.loads((result / "failed_conditions.json").read_text(encoding="utf-8"))
    assert manifest["planned_condition_count"] == 1
    assert manifest["state"] == "failed"
    assert manifest["completed_condition_count"] == 0
    assert manifest["failed_conditions"] == failures
    assert failures[0]["condition_id"] == "pid-only-medium-seed000"
    assert failures[0]["error_type"] == "RuntimeError"


@pytest.mark.parametrize(
    ("first_raises", "stable", "expected_state", "scientific_failures", "errors"),
    [
        (True, True, "partial", 0, 1),
        (False, False, "completed", 2, 0),
    ],
)
def test_ablation_protocol_preserves_partial_and_scientific_failure_semantics(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    first_raises: bool,
    stable: bool,
    expected_state: str,
    scientific_failures: int,
    errors: int,
) -> None:
    """协议集成层对部分异常和全科学失败保持公共生命周期语义。"""
    from parallel_gripper_tactile.studies.protocols import force_tracking_ablation as protocol

    source = REPOSITORY_ROOT / "configs/studies/smoke/force_tracking_ablation.yaml"
    config = load_study_config(source).model_copy(update={"controllers": ("pid-only", "full")})

    def fake_execute(**kwargs: object):
        if first_raises and kwargs["controller_variant"] == "pid-only":
            raise RuntimeError("synthetic partial failure")
        run_path = Path(str(kwargs["output_root"])) / f"{kwargs['run_prefix']}-synthetic"
        run_path.mkdir(parents=True)
        result = ForceTrackingResult(
            contact_time_s=0.1,
            tracking_start_time_s=0.2,
            tracking_duration_s=1.0,
            rmse_n=0.1,
            mae_n=0.1,
            peak_abs_error_n=0.2,
            mean_error_n=0.0,
            final_error_n=0.0,
            torque_saturation_ratio=0.0,
            position_saturation_ratio=0.0,
            mean_estimated_stiffness_n_per_m=1000.0,
            rise_time_s=0.1,
            overshoot_ratio=0.1,
            settling_time_s=0.2,
            simulation_stable=stable,
        )
        return SimpleNamespace(path=run_path), result

    monkeypatch.setattr(protocol, "execute_force_tracking", fake_execute)
    monkeypatch.setattr(protocol, "render_study_figures", lambda *args, **kwargs: [])
    protocol.run_study(config, config_source=source, study_directory=tmp_path)

    manifest = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == expected_state
    assert manifest["scientific_failure_count"] == scientific_failures
    assert manifest["execution_error_count"] == errors
    assert manifest["failed_condition_count"] == scientific_failures + errors


def test_comparison_protocol_continues_after_a_condition_exception(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """comparison 也在单条件异常后继续，并形成可用的 partial manifest。"""
    from parallel_gripper_tactile.studies.protocols import (
        force_tracking_controller_comparison as protocol,
    )

    source = REPOSITORY_ROOT / "configs/studies/force_tracking_controller_comparison.yaml"
    original = load_comparison_config(source)
    config = original.model_copy(
        update={
            "controllers": ("pid-only", "full"),
            "tasks": (original.tasks[0],),
            "materials": ("medium",),
            "seeds": original.seeds.model_copy(update={"count": 1}),
        }
    )

    def fake_execute(**kwargs: object):
        if kwargs["controller_variant"] == "pid-only":
            raise RuntimeError("synthetic comparison failure")
        run_path = Path(str(kwargs["output_root"])) / f"{kwargs['run_prefix']}-synthetic"
        run_path.mkdir(parents=True)
        result = ForceTrackingResult(
            contact_time_s=0.1,
            tracking_start_time_s=0.2,
            tracking_duration_s=1.0,
            rmse_n=0.1,
            mae_n=0.1,
            peak_abs_error_n=0.2,
            mean_error_n=0.0,
            final_error_n=0.0,
            torque_saturation_ratio=0.0,
            position_saturation_ratio=0.0,
            mean_estimated_stiffness_n_per_m=1000.0,
            rise_time_s=0.1,
            overshoot_ratio=0.1,
            settling_time_s=0.2,
            simulation_stable=True,
        )
        return SimpleNamespace(path=run_path), result

    monkeypatch.setattr(protocol, "execute_force_tracking", fake_execute)
    monkeypatch.setattr(protocol, "render_study_figures", lambda *args, **kwargs: [])
    protocol.run_study(config, config_source=source, study_directory=tmp_path)

    manifest = json.loads((tmp_path / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == "partial"
    assert manifest["completed_condition_count"] == 1
    assert manifest["execution_error_count"] == 1
    assert manifest["failed_conditions"][0]["controller_variant"] == "pid-only"


@pytest.mark.parametrize("argument", ["-m", "--multirun", "hydra.mode=MULTIRUN"])
def test_study_rejects_outer_multirun_before_hydra_dispatch(argument: str) -> None:
    """正式研究在 Hydra 创建任何 job 前拒绝外层矩阵展开。"""
    with pytest.raises(SystemExit, match="禁止 Hydra 外层 Multirun"):
        _entry_module().reject_outer_multirun([argument])
