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
from parallel_gripper_tactile.experiments.force_tracking import (
    ForceTrackingResult,
    ForceTrackingTask,
)
from parallel_gripper_tactile.research.hydra_support import register_resolvers, resolved_mapping
from parallel_gripper_tactile.research.study import (
    ResearchStudySetupError,
    execute_research_study,
    resolve_research_study,
)
from parallel_gripper_tactile.studies.force_tracking_comparison import load_comparison_config


CONFIG_ROOT = REPOSITORY_ROOT / "configs"
STUDY_GROUPS = {
    "force_tracking_controller_comparison": ("dm_force_controller_selection/study", []),
    "friction_estimator_validation": ("friction_estimator_validation/study", []),
    "force_tracking_stiffness_rate_confirmation": ("dm_stiffness_rate_confirmation/study", []),
    "force_scheduling_support_release": ("adaptive_support_release_validation/study", []),
}


@pytest.mark.parametrize(
    "research_group",
    [
        "dm_force_controller_selection/study",
        "friction_estimator_validation/study",
        "dm_stiffness_rate_confirmation/study",
        "adaptive_support_release_validation/study",
    ],
)
def test_active_research_groups_do_not_repeat_cross_layer_fields(
    research_group: str,
) -> None:
    """活跃研究不重复保存 profile、输出目录或矩阵占位选择。"""
    register_resolvers()
    with initialize_config_dir(version_base="1.3", config_dir=str(CONFIG_ROOT)):
        raw = resolved_mapping(
            compose(config_name="study", overrides=[f"research={research_group}"])
        )
    study = raw["study"]
    definition = study["definition"]
    assert "profile" not in definition
    assert "output_root" not in definition
    overrides = tuple(study["profile"].get("overrides", ()))
    assert not any(value.startswith(("task=", "seed=", "execution=")) for value in overrides)


def _resolved(name: str, *, overrides: list[str] | None = None):
    """组合并解析一个正式研究 preset。"""
    register_resolvers()
    research, defaults = STUDY_GROUPS[name]
    with initialize_config_dir(version_base="1.3", config_dir=str(CONFIG_ROOT)):
        config = compose(
            config_name="study",
            overrides=[f"research={research}", *defaults, *(overrides or [])],
        )
    return resolve_research_study(resolved_mapping(config))


def test_execution_workers_override_is_validated() -> None:
    """正式研究接受正整数进程数，并拒绝零进程。"""
    resolved = _resolved(
        "friction_estimator_validation",
        overrides=["execution.workers=4"],
    )
    assert resolved.selection.execution.workers == 4
    with pytest.raises(ResearchStudySetupError, match="greater than or equal to 1"):
        _resolved(
            "friction_estimator_validation",
            overrides=["execution.workers=0"],
        )


def test_plot_mode_preserves_scientific_plan_and_rejects_unknown_mode() -> None:
    """出图只影响工件选择，不改变科学哈希或有序实验条件。"""
    summary = _resolved("friction_estimator_validation")
    diagnostic = _resolved(
        "friction_estimator_validation",
        overrides=["execution.plot_mode=diagnostic"],
    )
    assert summary.plan == diagnostic.plan
    assert diagnostic.selection.execution.plot_mode == "diagnostic"
    assert (
        diagnostic.effective_configuration()["selection"]["execution"]["plot_mode"] == "diagnostic"
    )
    with pytest.raises(ResearchStudySetupError, match="plot_mode"):
        _resolved(
            "friction_estimator_validation",
            overrides=["execution.plot_mode=all"],
        )


def _migrated_task(path: Path) -> Path:
    """把冻结基线 task 路径映射到目标权威配置目录。"""
    relative = path.relative_to(REPOSITORY_ROOT / "configs")
    if relative.parts[0] == "task":
        return path
    name = "mixed.yaml" if relative.name == "mixed_waypoints.yaml" else relative.name
    return REPOSITORY_ROOT / "configs" / "task" / relative.parent / name


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
        REPOSITORY_ROOT / "configs/research/dm_force_controller_selection/study.yaml"
    )
    expected_controllers = (
        "pid-only",
        "pid-torque-ff",
        "pid-stiffness-rate",
        "adrc-torque",
    )

    actual = tuple(
        (row["controller"], Path(str(row["task"])), row["material"], row["seed"])
        for row in resolved.conditions
    )
    assert legacy.controllers == expected_controllers
    expected = tuple(
        (controller, _migrated_task(task), material, seed)
        for controller, task, material, seed in legacy.conditions()
    )
    assert actual == expected
    assert len(set(str(row["condition_id"]) for row in resolved.conditions)) == 108


def test_default_formal_selection_excludes_historical_low_performers() -> None:
    """默认选型矩阵排除 direct-torque，并在配置中保留退出证据。"""
    resolved = _resolved("force_tracking_controller_comparison")

    assert isinstance(resolved.domain_config, type(load_comparison_config(resolved.config_source)))
    assert "direct-torque" not in resolved.domain_config.controllers
    exclusions = resolved.selection.study.rationale.exclusions
    assert "direct-torque" in exclusions
    assert "135-run" in exclusions["direct-torque"]
    assert "archive" not in str(resolved.config_source.relative_to(CONFIG_ROOT))


def test_stiffness_rate_confirmation_preserves_250hz_material_matrix() -> None:
    """确认研究固定 250 Hz 与胜出候选，并完整覆盖材料和配对 seed。"""
    resolved = _resolved("force_tracking_stiffness_rate_confirmation")
    config = resolved.domain_config

    assert config.analysis_mode == "confirmation"
    assert tuple(ForceTrackingTask.load(task).control_period_s for task in config.tasks) == (0.004,)
    assert config.materials == ("medium", "hard", "stiff")
    assert config.kp_s_inv == (20.0,)
    assert config.max_force_rate_n_s == (50.0,)
    assert resolved.plan.study_kind == "force_tracking_stiffness_rate_confirmation"
    assert len(resolved.conditions) == 18
    assert len({row["pair_key"] for row in resolved.conditions}) == 9
    assert sum(row["baseline_role"] == "performance-baseline" for row in resolved.conditions) == 9


@pytest.mark.parametrize("research_group", ["dm_force_controller_selection/study"])
def test_retired_diagnosis_phase_is_rejected_before_preflight(research_group: str) -> None:
    """退役的诊断阶段字段不能被活跃研究静默忽略。"""
    register_resolvers()
    with initialize_config_dir(version_base="1.3", config_dir=str(CONFIG_ROOT)):
        raw = resolved_mapping(
            compose(config_name="study", overrides=[f"research={research_group}"])
        )
    raw["study"]["phase"] = "collision-geometry"
    with pytest.raises(ResearchStudySetupError, match="Extra inputs are not permitted") as caught:
        resolve_research_study(raw)
    assert caught.value.stage == "configuration"


def test_retired_diagnosis_kind_is_rejected_before_preflight() -> None:
    """旧诊断配置不能误走其他研究的计划或执行路径。"""
    register_resolvers()
    with initialize_config_dir(version_base="1.3", config_dir=str(CONFIG_ROOT)):
        raw = resolved_mapping(compose(config_name="study"))
    raw["study"]["kind"] = "force_tracking_diagnosis"
    with pytest.raises(ResearchStudySetupError, match="Input should be") as caught:
        resolve_research_study(raw)
    assert caught.value.stage == "configuration"


def test_formal_friction_estimator_validation_preserves_scenario_and_seed_matrix() -> None:
    """迁移后的 15 条局部起滑条件与旧 study 配置逐项一致。"""
    from parallel_gripper_tactile.studies.friction_estimator_validation import (
        load_friction_estimator_validation_config,
    )

    resolved = _resolved("friction_estimator_validation")
    legacy = load_friction_estimator_validation_config(
        REPOSITORY_ROOT / "configs/research/friction_estimator_validation/study.yaml"
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
        (_migrated_task(scenario.task), scenario.expect_local_slip, seed)
        for scenario, seed in legacy.conditions()
    )
    assert actual == expected
    assert len({row["condition_id"] for row in resolved.conditions}) == 15


@pytest.mark.parametrize(
    ("study_name", "condition_count"),
    [
        ("friction_estimator_validation", 15),
        ("force_scheduling_support_release", 18),
    ],
)
def test_plan_registers_expected_condition_count(
    tmp_path: Path, study_name: str, condition_count: int
) -> None:
    """各正式 study 的静态计划登记条件数且不创建任何 run。"""
    resolved = _resolved(study_name)
    result = execute_research_study(
        resolved,
        hydra_output_directory=tmp_path,
        provenance={"choices": {}, "overrides": []},
    )

    plan = json.loads((result / "plan.json").read_text(encoding="utf-8"))
    assert plan["study"] == study_name
    assert plan["condition_count"] == condition_count
    manifest = json.loads((result / "study_manifest.json").read_text(encoding="utf-8"))
    assert manifest["state"] == "planned"
    assert not (result / "runs").exists()


def test_friction_protocol_preserves_validation_semantics(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """局部起滑以 validation_passed 作为科学验收并保留期望汇总字段。"""
    from parallel_gripper_tactile.experiments.friction_estimation import (
        FrictionEstimationResult,
        FrictionEstimationTask,
    )
    from parallel_gripper_tactile.studies.friction_estimator_validation import (
        load_friction_estimator_validation_config,
    )
    from parallel_gripper_tactile.studies.protocols import (
        friction_estimator_validation as protocol,
    )

    source = REPOSITORY_ROOT / "configs/research/friction_estimator_validation/study.yaml"
    original = load_friction_estimator_validation_config(source)
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


def test_friction_protocol_records_condition_exceptions(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """单条件异常仍生成完整失败账本，不伪装成未执行计划。"""
    from parallel_gripper_tactile.experiments.friction_estimation import (
        FrictionEstimationTask,
    )
    from parallel_gripper_tactile.studies.friction_estimator_validation import (
        load_friction_estimator_validation_config,
    )
    from parallel_gripper_tactile.studies.protocols import (
        friction_estimator_validation as protocol,
    )

    source = REPOSITORY_ROOT / "configs/research/friction_estimator_validation/study.yaml"
    original = load_friction_estimator_validation_config(source)
    config = original.model_copy(
        update={
            "scenarios": original.scenarios[:1],
            "seeds": original.seeds.model_copy(update={"count": 1}),
        }
    )

    def fail(*args: object, **kwargs: object):
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
    resolved = _resolved("friction_estimator_validation")
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


def test_study_hash_ignores_output_root_but_changes_with_science() -> None:
    """输出位置不进入科学哈希，而条件矩阵变化必须改变哈希。"""
    from parallel_gripper_tactile.studies.friction_estimator_validation import (
        load_friction_estimator_validation_config,
    )
    from parallel_gripper_tactile.studies.protocols import (
        friction_estimator_validation as protocol,
    )

    config = load_friction_estimator_validation_config(
        REPOSITORY_ROOT / "configs/research/friction_estimator_validation/study.yaml"
    )
    original = protocol.build_plan(config)
    moved = protocol.build_plan(
        config.model_copy(update={"output_root": Path("/tmp/unrelated-output")})
    )
    changed = protocol.build_plan(config.model_copy(update={"scenarios": config.scenarios[:1]}))
    assert moved.scientific_configuration_sha256 == original.scientific_configuration_sha256
    assert changed.scientific_configuration_sha256 != original.scientific_configuration_sha256


def test_study_hash_is_independent_of_current_working_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """同一已解析科学配置从不同 cwd 生成相同哈希。"""
    from parallel_gripper_tactile.studies.friction_estimator_validation import (
        load_friction_estimator_validation_config,
    )
    from parallel_gripper_tactile.studies.protocols import (
        friction_estimator_validation as protocol,
    )

    config = load_friction_estimator_validation_config(
        REPOSITORY_ROOT / "configs/research/friction_estimator_validation/study.yaml"
    )
    expected = protocol.build_plan(config).scientific_configuration_sha256
    monkeypatch.chdir(tmp_path)
    actual = protocol.build_plan(config).scientific_configuration_sha256
    assert actual == expected


def test_execution_passes_the_exact_resolved_plan_to_protocol(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """执行层不重新展开条件，直接传递解析阶段已校验的计划实例。"""
    from parallel_gripper_tactile.studies.protocols import (
        friction_estimator_validation as protocol,
    )

    resolved = _resolved("friction_estimator_validation")
    run_selection = resolved.selection.model_copy(
        update={
            "execution": resolved.selection.execution.model_copy(
                update={"mode": "run", "workers": 3}
            )
        }
    )
    runnable = replace(resolved, selection=run_selection)
    captured: dict[str, object] = {}

    def fake_run(config: object, **kwargs: object) -> Path:
        captured.update(kwargs)
        return Path(str(kwargs["study_directory"]))

    monkeypatch.setattr(protocol, "run_study", fake_run)
    updates = []
    on_progress = updates.append
    execute_research_study(
        runnable,
        hydra_output_directory=tmp_path,
        provenance={"choices": {}, "overrides": []},
        on_progress=on_progress,
    )
    assert captured["study_plan"] is resolved.plan
    assert captured["resolved_profile"] is resolved.profile
    assert captured["workers"] == 3
    assert captured["plot_mode"] == "summary"
    assert captured["on_progress"] is on_progress
    assert updates == []
    assert {Path(path).name for path in captured["additional_artifacts"]} == {
        "effective_study_configuration.json",
        "composition_provenance.json",
    }


def test_protocol_records_condition_exceptions_and_finishes_manifest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """单条件异常仍生成完整失败账本，不伪装成未执行计划。"""
    from parallel_gripper_tactile.studies.friction_estimator_validation import (
        load_friction_estimator_validation_config,
    )
    from parallel_gripper_tactile.studies.protocols import (
        friction_estimator_validation as protocol,
    )

    original = load_friction_estimator_validation_config(
        REPOSITORY_ROOT / "configs/research/friction_estimator_validation/study.yaml"
    )
    config = original.model_copy(
        update={
            "scenarios": original.scenarios[:1],
            "seeds": original.seeds.model_copy(update={"count": 1}),
        }
    )

    def fail(*args: object, **kwargs: object):
        raise RuntimeError("synthetic failure")

    monkeypatch.setattr(protocol, "_execute_condition", fail)
    result = protocol.run_study(
        config,
        config_source=REPOSITORY_ROOT / "configs/research/friction_estimator_validation/study.yaml",
        study_directory=tmp_path,
    )

    manifest = json.loads((result / "study_manifest.json").read_text(encoding="utf-8"))
    failures = json.loads((result / "failed_conditions.json").read_text(encoding="utf-8"))
    assert manifest["planned_condition_count"] == 1
    assert manifest["state"] == "failed"
    assert manifest["completed_condition_count"] == 0
    assert manifest["failed_conditions"] == failures
    assert failures[0]["condition_id"].endswith("-seed000")
    assert failures[0]["error_type"] == "RuntimeError"


def test_comparison_protocol_continues_after_a_condition_exception(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """comparison 也在单条件异常后继续，并形成可用的 partial manifest。"""
    from parallel_gripper_tactile.studies.protocols import (
        force_tracking_controller_comparison as protocol,
    )

    source = REPOSITORY_ROOT / "configs/research/dm_force_controller_selection/study.yaml"
    original = load_comparison_config(source)
    config = original.model_copy(
        update={
            "controllers": ("pid-only", "pid-torque-ff"),
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


@pytest.mark.parametrize("plot_mode", ["summary", "diagnostic"])
def test_study_selects_diagnostic_seed_before_results(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, plot_mode: str
) -> None:
    """代表 seed 由计划决定，首条件失败不会把后续 seed 改选为代表。"""
    from parallel_gripper_tactile.studies.friction_estimator_validation import (
        load_friction_estimator_validation_config,
    )
    from parallel_gripper_tactile.studies.protocols import (
        friction_estimator_validation as protocol,
    )

    source = REPOSITORY_ROOT / "configs/research/friction_estimator_validation/study.yaml"
    original = load_friction_estimator_validation_config(source)
    config = original.model_copy(
        update={
            "scenarios": original.scenarios[:1],
            "seeds": original.seeds.model_copy(update={"start": 7, "count": 2}),
        }
    )
    seen = {}

    def fail(condition, *args: object, **kwargs: object):
        seen[condition.parameters["sensor_noise_seed"]] = kwargs["diagnostic_seed"]
        raise RuntimeError("仅核对计划派发，不启动仿真。")

    monkeypatch.setattr(protocol, "_execute_condition", fail)
    protocol.run_study(config, config_source=source, study_directory=tmp_path, plot_mode=plot_mode)
    # 代表 seed 在计划中固定，条件失败不会改选后续 seed 的诊断目标。
    assert seen == {7: 7, 8: 7}
