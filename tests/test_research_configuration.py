"""验证统一 ``run.yaml`` 的 Hydra 组合与冻结领域对象边界。"""

from __future__ import annotations

import json
from pathlib import Path

from hydra import compose, initialize_config_dir
from hydra.errors import ConfigCompositionException
from omegaconf import OmegaConf
import pytest

from parallel_gripper_tactile.research import (
    REPOSITORY_ROOT,
    ResearchConfigurationError,
    compose_research_run,
    resolve_research_run,
)
from parallel_gripper_tactile.research.hydra_support import register_resolvers, resolved_mapping


CONFIG_ROOT = REPOSITORY_ROOT / "configs"
BASELINE_PATH = REPOSITORY_ROOT / "tests/baselines/configuration_refactor_v1.json"


def _compose(overrides: list[str] | None = None):
    """从真实目标配置目录组合统一入口。"""
    register_resolvers()
    with initialize_config_dir(version_base="1.3", config_dir=str(CONFIG_ROOT)):
        return compose(config_name="run", overrides=overrides or [])


def _baseline(name: str) -> dict[str, object]:
    """读取迭代 1 冻结的旧入口有效参数快照。"""
    snapshot = json.loads(BASELINE_PATH.read_text(encoding="utf-8"))
    return snapshot["single_runs"][name]["effective"]


def _migration_snapshot() -> dict[str, object]:
    """读取完整迁移快照。"""
    return json.loads(BASELINE_PATH.read_text(encoding="utf-8"))


def _with_repository_root(value: object) -> object:
    """将基线的可移植仓库占位符还原为当前绝对路径。"""
    if isinstance(value, dict):
        return {key: _with_repository_root(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_with_repository_root(item) for item in value]
    if isinstance(value, str):
        return value.replace("${REPOSITORY_ROOT}", str(REPOSITORY_ROOT))
    return value


def _with_current_dm_supervisor(value: object) -> object:
    """为冻结旧快照补入当前 DMgripper 公共接触状态机配置。"""
    restored = _with_repository_root(value)
    if isinstance(restored, dict):
        control = restored.get("control")
        if isinstance(control, dict):
            force = control.get("force")
            if isinstance(force, dict):
                if restored.get("name") == "dm_gripper_admittance":
                    force["contact_threshold_n"] = 0.15
                force.setdefault(
                    "supervisor",
                    {
                        "contact_stable_time_s": 0.0,
                        "contact_transition_time_s": 0.05,
                        "release_policy": "any_side",
                    },
                )
    return restored


def test_default_run_matches_frozen_legacy_domain_parameters() -> None:
    """默认统一入口的 profile 与 task 保持迭代 1 基线等价。"""
    resolved = resolve_research_run(resolved_mapping(_compose()))
    expected = _baseline("dm_force_track")

    assert resolved.profile.model_dump(mode="json") == _with_current_dm_supervisor(
        expected["profile"]
    )
    assert resolved.task.model_dump(mode="json") == expected["task"]
    assert resolved.selection.model.name == "height_spheres"
    assert resolved.selection.execution.trace_sample_period_s == 0.004


def test_admittance_combination_matches_frozen_legacy_domain_parameters() -> None:
    """导纳片段、显式关闭估计器与导纳 task 保持完整领域参数等价。"""
    resolved = resolve_research_run(
        resolved_mapping(
            _compose(
                [
                    "experiment=dm_gripper/force_tracking_admittance",
                ]
            )
        )
    )
    expected = _baseline("dm_admittance")

    assert resolved.profile.model_dump(mode="json") == _with_current_dm_supervisor(
        expected["profile"]
    )
    assert resolved.task.model_dump(mode="json") == expected["task"]
    assert resolved.profile.normal_force is not None
    assert resolved.profile.normal_force.admittance is not None
    assert resolved.profile.normal_force.stiffness is not None
    assert resolved.profile.normal_force.stiffness.enabled is False
    assert resolved.selection.execution.trace_sample_period_s == resolved.task.control_period_s


@pytest.mark.parametrize(
    "model_name, expected_resource",
    [
        ("height_spheres", "parallel_gripper_height_sphere_collision.xml"),
        ("flat_spheres", "parallel_gripper_flat_sphere_collision.xml"),
        ("coplanar_mesh", "parallel_gripper_coplanar_mesh_collision.xml"),
        ("original_mesh", "parallel_gripper_prepared.xml"),
    ],
)
def test_model_group_switches_only_the_selected_collision_resource(
    model_name: str, expected_resource: str
) -> None:
    """四种 DM 碰撞模型都能在计划预检中完成 scene 编译。"""
    resolved = resolve_research_run(
        resolved_mapping(_compose([f"model=dm_gripper/{model_name}", "execution=plan"]))
    )

    assert resolved.selection.model.name == model_name
    assert resolved.profile.model_path.name == expected_resource


def test_flat_model_preserves_legacy_identity_and_changes_no_control_fields() -> None:
    """平面球体选择保持旧 profile 名，并且只替换模型资源。"""
    default = resolve_research_run(resolved_mapping(_compose()))
    flat = resolve_research_run(resolved_mapping(_compose(["model=dm_gripper/flat_spheres"])))
    default_mapping = default.profile.model_dump(mode="json")
    flat_mapping = flat.profile.model_dump(mode="json")

    assert flat.profile.name == "dm_gripper_flat_spheres"
    assert flat_mapping["control"] == default_mapping["control"]
    assert flat_mapping["mount"] == default_mapping["mount"]
    assert flat_mapping["tactile"] == default_mapping["tactile"]


def test_estimator_group_replaces_its_fragment_and_reaches_final_profile() -> None:
    """估计器切换只改变其方法字段，并进入最终冻结 profile。"""
    resolved = resolve_research_run(
        resolved_mapping(_compose(["controller=dm_gripper/full", "estimator=window_quadratic"]))
    )

    assert resolved.profile.normal_force is not None
    assert resolved.profile.normal_force.stiffness is not None
    assert resolved.profile.normal_force.stiffness.method == "window_quadratic"


@pytest.mark.parametrize(
    "controller",
    [
        "pid_only",
        "pid_torque_ff",
        "pid_stiffness_ff",
        "pid_stiffness_limit",
        "full",
        "direct_torque",
        "adrc",
        "adrc_torque",
        "adrc_torque_td",
    ],
)
def test_all_dm_non_admittance_controller_groups_resolve(controller: str) -> None:
    """全部 DM PID／ADRC 与独立复现控制器均能形成严格领域对象。"""
    resolved = resolve_research_run(
        resolved_mapping(_compose([f"controller=dm_gripper/{controller}"]))
    )

    assert resolved.profile.normal_force is not None


@pytest.mark.parametrize(
    ("experiment", "task_group", "legacy_task"),
    [
        (
            "dm_gripper/force_scheduling_gravity_hold",
            "force_scheduling/gravity_hold",
            "configs/force_scheduling/gravity_hold.yaml",
        ),
        (
            "dm_gripper/force_scheduling_dynamic_filling",
            "force_scheduling/dynamic_filling",
            "configs/force_scheduling/dynamic_filling.yaml",
        ),
        (
            "dm_gripper/friction_estimation_nominal",
            "friction_estimation/nominal_friction",
            "configs/friction_estimation/nominal_friction.yaml",
        ),
        (
            "dm_gripper/friction_estimation_nominal",
            "friction_estimation/low_friction",
            "configs/friction_estimation/low_friction.yaml",
        ),
        (
            "dm_gripper/friction_estimation_nominal",
            "friction_estimation/high_friction",
            "configs/friction_estimation/high_friction.yaml",
        ),
        (
            "dm_gripper/friction_estimation_nominal",
            "friction_estimation/noisy_friction",
            "configs/friction_estimation/noisy_friction.yaml",
        ),
        (
            "dm_gripper/friction_estimation_nominal",
            "friction_estimation/no_slip_low_probe",
            "configs/friction_estimation/no_slip_low_probe.yaml",
        ),
        (
            "dm_gripper/friction_estimation_nominal",
            "friction_estimation/hardware_scale_nominal",
            "configs/friction_estimation/hardware_scale_nominal.yaml",
        ),
        (
            "robotiq_2f85/discrete_force",
            "discrete_force/robotiq_delta_f_tick",
            "configs/discrete_force/robotiq_delta_f_tick.yaml",
        ),
    ],
)
def test_remaining_task_groups_match_frozen_domain_values(
    experiment: str, task_group: str, legacy_task: str
) -> None:
    """力调度、摩擦估计和离散力 task 保持迁移前完整领域值。"""
    resolved = resolve_research_run(
        resolved_mapping(
            _compose([f"experiment={experiment}", f"task={task_group}", "execution=plan"])
        )
    )
    snapshot = _migration_snapshot()
    family = legacy_task.split("/")[1]
    expected = snapshot["tasks"][family][legacy_task]["effective"]

    assert resolved.task.model_dump(mode="json") == expected


@pytest.mark.parametrize(
    ("model_group", "legacy_profile"),
    [
        ("sphere_force_sensor", "configs/robotiq_2f85.yaml"),
        ("box_force_sensor", "configs/robotiq_2f85_box.yaml"),
        ("touch_grid_3x3", "configs/robotiq_2f85_touch_grid.yaml"),
    ],
)
def test_robotiq_model_groups_match_frozen_profiles(model_group: str, legacy_profile: str) -> None:
    """Robotiq 三种 model 与相容命令上限、触觉布局逐字段等价。"""
    resolved = resolve_research_run(
        resolved_mapping(
            _compose(
                [
                    "experiment=robotiq_2f85/discrete_force",
                    f"model=robotiq_2f85/{model_group}",
                    "execution=plan",
                ]
            )
        )
    )
    expected = _migration_snapshot()["profiles"][legacy_profile]["effective"]

    assert resolved.profile.model_dump(mode="json") == _with_repository_root(expected)


@pytest.mark.parametrize(
    "overrides",
    [
        ["controller=dm_gripper/admittance"],
        ["estimator=none"],
        ["controller.torque_adrc.measurement_filter_cutoff_hz=-1"],
    ],
)
def test_illegal_component_combinations_fail_before_execution(overrides: list[str]) -> None:
    """不兼容选择和非法范围均在生成最终配置时失败。"""
    with pytest.raises(ResearchConfigurationError):
        resolve_research_run(resolved_mapping(_compose(overrides)))


def test_unknown_override_is_rejected_by_hydra() -> None:
    """拼错的运行时覆盖不会静默加入任务配置。"""
    with pytest.raises(ConfigCompositionException):
        _compose(["materail=hard"])


def test_resource_resolution_does_not_depend_on_process_cwd(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """从其他工作目录解析不会改变 profile、task 或 MJCF 资源。"""
    raw = resolved_mapping(_compose(["execution=plan"]))
    first = resolve_research_run(raw)
    monkeypatch.chdir(tmp_path)
    second = resolve_research_run(raw)

    assert first.profile_source == second.profile_source
    assert first.task_source == second.task_source
    assert first.profile.model_path == second.profile.model_path
    assert first.effective_parameters() == second.effective_parameters()


def test_programmatic_composition_service_is_cwd_independent(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """CLI 等非 Hydra 入口也通过同一服务组合命名实验与覆盖。"""
    first = compose_research_run(
        experiment="robotiq_2f85/discrete_force",
        overrides=("model=robotiq_2f85/touch_grid_3x3", "execution=plan"),
    )
    monkeypatch.chdir(tmp_path)
    second = compose_research_run(
        experiment="robotiq_2f85/discrete_force",
        overrides=("model=robotiq_2f85/touch_grid_3x3", "execution=plan"),
    )

    assert first.profile == second.profile
    assert first.task == second.task
    assert first.selection == second.selection


def test_programmatic_composition_reuses_an_active_hydra_context() -> None:
    """正式 study 在 Hydra job 内仍可调用同一单次组合服务。"""
    register_resolvers()
    with initialize_config_dir(version_base="1.3", config_dir=str(CONFIG_ROOT)):
        resolved = compose_research_run(
            experiment="dm_gripper/force_tracking_admittance",
            overrides=("execution=plan",),
        )

    assert resolved.profile.name == "dm_gripper_admittance"
    assert resolved.selection.execution.mode == "plan"


def test_resolved_mapping_rejects_unresolved_missing_value() -> None:
    """OmegaConf 的强制缺失值在进入 Pydantic 前失败。"""
    config = OmegaConf.create({"required": "???"})
    with pytest.raises(Exception, match="Missing mandatory value"):
        resolved_mapping(config)
