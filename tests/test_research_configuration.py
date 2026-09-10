"""验证 Hydra 组合配置与领域最终配置的严格边界。"""

from __future__ import annotations

from pathlib import Path

from hydra import compose, initialize_config_dir
from hydra.errors import ConfigCompositionException
from omegaconf import OmegaConf
import pytest

from parallel_gripper_tactile.config.profiles import load_profile
from parallel_gripper_tactile.experiments.force_tracking import (
    ForceTrackingTask,
    configure_force_controller,
)
from parallel_gripper_tactile.research import (
    REPOSITORY_ROOT,
    ResearchConfigurationError,
    resolve_research_run,
)
from parallel_gripper_tactile.research.hydra_support import register_resolvers, resolved_mapping


CONFIG_ROOT = REPOSITORY_ROOT / "configs" / "research"


def _compose(name: str = "dm_force_track", overrides: list[str] | None = None):
    """从真实配置目录组合一个测试配置。"""
    register_resolvers()
    with initialize_config_dir(version_base="1.3", config_dir=str(CONFIG_ROOT)):
        return compose(config_name=name, overrides=overrides or [])


def test_groups_replace_complete_components_and_resolve_interpolation() -> None:
    """组选择整体替换控制器、估计器、任务和平台，路径插值完全解析。"""
    config = _compose(
        overrides=[
            "platform=dm/admittance_simulation",
            "controller=dm/admittance",
            "estimator=none",
            "task=dm_admittance",
            "material=medium",
            "seed=7",
            "execution=plan",
        ]
    )
    raw = resolved_mapping(config)
    resolved = resolve_research_run(raw)

    assert resolved.selection.controller.name == "admittance"
    assert resolved.selection.controller.torque_adrc is None
    assert resolved.selection.estimator.name == "none"
    assert resolved.selection.material.name == "medium"
    assert resolved.profile.normal_force is not None
    assert resolved.profile.normal_force.admittance is not None
    assert resolved.profile.normal_force.adrc is None
    assert resolved.profile.normal_force.torque_adrc is None
    assert resolved.profile.normal_force.stiffness is not None
    assert resolved.profile.normal_force.stiffness.enabled is False
    assert resolved.selection.execution.output_root.is_absolute()
    assert resolved.selection.execution.trace_sample_period_s == resolved.task.control_period_s


def test_adrc_group_has_only_its_validated_parameters() -> None:
    """ADRC 组显式参数进入最终 profile，其他算法字段被清空。"""
    resolved = resolve_research_run(resolved_mapping(_compose()))
    force = resolved.profile.normal_force

    assert force is not None
    assert force.torque_adrc == resolved.selection.controller.torque_adrc
    assert force.adrc is None
    assert force.admittance is None
    assert force.torque_feedback_gain == 0.0
    assert resolved.selection.execution.trace_sample_period_s == 0.004


def test_runtime_field_overrides_take_priority_and_reach_the_final_profile() -> None:
    """命令行字段覆盖优先于 preset，并写入实际执行使用的最终对象。"""
    resolved = resolve_research_run(
        resolved_mapping(
            _compose(
                overrides=[
                    "controller.torque_adrc.controller_bandwidth_rad_s=55.0",
                    "execution.trace_sample_period_s=0.008",
                    "seed=9",
                ]
            )
        )
    )

    assert resolved.selection.controller.torque_adrc is not None
    assert resolved.selection.controller.torque_adrc.controller_bandwidth_rad_s == 55.0
    assert resolved.profile.normal_force is not None
    assert resolved.profile.normal_force.torque_adrc is not None
    assert resolved.profile.normal_force.torque_adrc.controller_bandwidth_rad_s == 55.0
    assert resolved.profile.normal_force.sensor_noise_seed == 9
    assert resolved.selection.execution.trace_sample_period_s == 0.008


def test_controller_reconfiguration_clears_stale_algorithm_fields() -> None:
    """从 ADRC 切回 PID 时不会残留算法专用字段。"""
    base = load_profile(REPOSITORY_ROOT / "configs/dm_gripper.yaml")
    adrc = configure_force_controller(base, variant="adrc-torque")
    pid = configure_force_controller(adrc, variant="pid-only")

    assert pid.normal_force is not None
    assert pid.normal_force.adrc is None
    assert pid.normal_force.torque_adrc is None
    assert pid.normal_force.admittance is None
    assert pid.normal_force.torque_feedback_gain == 0.0


def test_hydra_and_legacy_resolution_produce_the_same_effective_inputs() -> None:
    """同一科学设置经新旧入口解析后得到完全相同的 profile 与 task。"""
    overrides = [
        "controller=dm/full",
        "estimator=window_linear",
        "task=step",
        "material=medium",
        "seed=0",
    ]
    resolved = resolve_research_run(resolved_mapping(_compose(overrides=overrides)))
    legacy_profile = configure_force_controller(
        load_profile(REPOSITORY_ROOT / "configs/dm_gripper.yaml"),
        variant="full",
        stiffness_estimator_method="window_linear",
        sensor_noise_seed=0,
    )

    assert resolved.profile == legacy_profile
    assert resolved.task == ForceTrackingTask.load(
        REPOSITORY_ROOT / "configs/force_tracking/step.yaml"
    )


@pytest.mark.parametrize(
    "overrides",
    [
        ["controller=dm/admittance"],
        ["estimator=none"],
        ["controller.torque_adrc.measurement_filter_cutoff_hz=-1"],
    ],
)
def test_illegal_component_combinations_fail_before_execution(overrides: list[str]) -> None:
    """不兼容选择和非法范围均在生成最终配置时失败。"""
    with pytest.raises(ResearchConfigurationError):
        resolve_research_run(resolved_mapping(_compose(overrides=overrides)))


def test_unknown_override_is_rejected_by_hydra() -> None:
    """拼错的运行时覆盖不会静默加入任务配置。"""
    with pytest.raises(ConfigCompositionException):
        _compose(overrides=["materail=hard"])


def test_missing_component_is_rejected_by_pydantic() -> None:
    """缺少完整组件时领域 schema 明确失败。"""
    raw = resolved_mapping(_compose())
    del raw["task"]
    with pytest.raises(ResearchConfigurationError, match="task"):
        resolve_research_run(raw)


def test_resource_resolution_does_not_depend_on_process_cwd(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """从其他工作目录解析不会改变 profile、task 或 MJCF 资源。"""
    raw = resolved_mapping(_compose(overrides=["execution=plan"]))
    first = resolve_research_run(raw)
    monkeypatch.chdir(tmp_path)
    second = resolve_research_run(raw)

    assert first.profile_source == second.profile_source
    assert first.task_source == second.task_source
    assert first.profile.model_path == second.profile.model_path
    assert first.effective_parameters() == second.effective_parameters()


def test_resolved_mapping_rejects_unresolved_missing_value() -> None:
    """OmegaConf 的强制缺失值在进入 Pydantic 前失败。"""
    config = OmegaConf.create({"required": "???"})
    with pytest.raises(Exception, match="Missing mandatory value"):
        resolved_mapping(config)
