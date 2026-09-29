"""通用实验配置的构造、严格 YAML 与跨字段校验测试。"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
from pathlib import Path

import pytest

from dmgripper_experiments.config import (
    AdmittanceConfig,
    AdaptiveReferenceConfig,
    ControllerConfig,
    EstimationConfig,
    ExperimentConfig,
    HardwareConfig,
    LifecycleConfig,
    RecordingConfig,
    SafetyConfig,
    StiffnessPreloadConfig,
    TimingConfig,
    UnifiedFrictionConfig,
    UnifiedHardwareConfig,
    UnifiedObserverConfig,
    load_experiment_config,
    sanitize_directory_component,
)
from dm_grasp_core.grasp.friction_depth import DepthFrictionPriorConfig
from dm_grasp_core.grasp.friction_particle import ParticleFrictionConfig
from dm_grasp_core.grasp.stiffness_adaptation import StiffnessAdmittanceConfig

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
REAL_CONFIGS = (
    REPOSITORY_ROOT / "configs/hardware/dmgripper/unified_adaptive.yaml",
    REPOSITORY_ROOT / "configs/hardware/dmgripper/unified_adaptive_particle.yaml",
)


def _unified_reference(estimator: str = "classic") -> AdaptiveReferenceConfig:
    """返回已授权闭环的统一自适应目标来源。"""
    return AdaptiveReferenceConfig(
        experimental_closed_loop=True,
        unified=UnifiedHardwareConfig(estimator=estimator),
    )


def test_default_config_constructs_without_unified():
    """默认配置可离线构造，默认不启用统一自适应闭环。"""
    config = ExperimentConfig()
    assert config.reference.unified is None
    assert not config.unified_adaptive_enabled
    assert config.reference.initial_force_n == pytest.approx(1.0)
    assert config.reference.duration_s is None
    assert config.estimation.enabled is True
    assert config.hardware.home_position_rad == pytest.approx(0.0)
    assert config.hardware.home_tolerance_rad == pytest.approx(0.03)
    assert config.hardware.feedback_position_margin_rad == pytest.approx(0.05)
    assert config.controller.admittance.feedforward_ratio == pytest.approx(1.0)
    assert config.controller.velocity_limit_rad_s == pytest.approx(0.3)
    assert config.controller.torque_limit_nm == pytest.approx(4.0)
    assert config.controller.zero_tracking_velocity is True
    assert config.controller.closing_torque_only is True


@pytest.mark.parametrize("path", REAL_CONFIGS)
def test_real_yaml_profiles_load(path: Path):
    """两份真实 YAML 都能通过严格校验加载。"""
    config = load_experiment_config(path)
    assert config.unified_adaptive_enabled
    assert config.reference.experimental_closed_loop
    assert config.lifecycle.on_finished == "hold"
    assert config.reference.duration_s is None
    assert config.estimation.enabled
    assert config.safety.max_target_force_n < config.safety.force_ceiling_n


def test_real_unified_profile_uses_depth_prior_estimator():
    """主力 profile 用深度先验估计器，统一策略权限按硬件视图固定。"""
    config = load_experiment_config(REAL_CONFIGS[0])
    unified = config.reference.unified
    assert unified is not None
    assert unified.estimator == "depth_prior"
    assert unified.particle_friction is None
    assert unified.depth_friction_prior is not None
    assert unified.depth_friction_prior.max_friction == pytest.approx(0.8)
    assert unified.depth_friction_prior.max_increase_per_s is None
    core = config.unified_core_config
    assert core.risk_enabled
    assert not core.risk_step_enabled
    assert core.friction_update_enabled
    assert core.depth_friction_prior is not None


def test_real_particle_profile_keeps_stiffness_preload_delta():
    """粒子 profile 保留刚度预载与粒子后验差异，并收紧时序。"""
    config = load_experiment_config(REAL_CONFIGS[1])
    unified = config.reference.unified
    assert unified is not None
    assert unified.estimator == "particle"
    assert unified.particle_friction is not None
    assert unified.particle_friction.seed == 23
    assert unified.depth_friction_prior is not None
    assert unified.friction.safety_factor == pytest.approx(1.2)
    assert unified.friction.min_event_taxels == 4
    assert unified.observer.min_normal_n == pytest.approx(0.08)
    assert config.timing.control_rate_hz == pytest.approx(100.0)
    assert config.timing.max_control_gap_s == pytest.approx(0.15)
    assert config.reference.initial_force_n == pytest.approx(0.5)
    assert config.safety.max_contact_compression_m == pytest.approx(0.020)
    assert config.controller.zero_tracking_velocity is False
    preload = config.lifecycle.stiffness_preload
    assert preload is not None
    assert preload.min_force_n == pytest.approx(0.5)
    assert preload.probe_force_n == pytest.approx(0.5)
    assert config.controller.admittance.stiffness_adaptation is not None


@pytest.mark.parametrize("path", REAL_CONFIGS)
def test_unified_core_config_is_derived_from_safety_single_source(path: Path):
    """core 统一策略包络由 safety 与初始目标唯一派生。"""
    config = load_experiment_config(path)
    unified = config.reference.unified
    assert unified is not None
    core = config.unified_core_config
    assert core.load.min_force_n == pytest.approx(config.reference.initial_force_n)
    assert core.load.max_force_n == pytest.approx(config.safety.max_target_force_n)
    assert core.load.max_force_rate_n_s == pytest.approx(config.safety.max_force_rate_n_s)
    assert core.load.max_force_decrease_rate_n_s == pytest.approx(
        config.safety.max_force_decrease_rate_n_s
    )
    assert core.observer.max_gap_s == pytest.approx(unified.max_sample_gap_s)
    assert core.observer.tangential_range_n == pytest.approx(4.0)
    assert core.observer.normal_range_n == pytest.approx(15.0)
    assert core.observer.contact_release_ratio == pytest.approx(float(Fraction(2, 3)))
    assert core.tracking_error_n == pytest.approx(unified.tracking_error_n)
    assert core.failure_timeout_s == pytest.approx(unified.failure_timeout_s)


def test_unified_core_config_requires_enabled_unified():
    """未启用统一模式时访问派生配置必须报错。"""
    with pytest.raises(ValueError, match="统一自适应"):
        ExperimentConfig().unified_core_config


def test_unified_requires_experimental_closed_loop_authorization():
    """任何统一启用都必须显式授权未验收闭环实验。"""
    with pytest.raises(ValueError, match="experimental_closed_loop"):
        ExperimentConfig(reference=AdaptiveReferenceConfig(unified=UnifiedHardwareConfig()))


@pytest.mark.parametrize(
    ("estimator", "with_depth", "with_particle"),
    [
        ("classic", True, False),
        ("classic", False, True),
        ("classic", True, True),
        ("depth_prior", False, False),
        ("depth_prior", True, True),
        ("particle", False, False),
    ],
)
def test_estimator_declarations_are_mutually_exclusive(
    estimator: str, with_depth: bool, with_particle: bool
):
    """estimator 判别与深度先验／粒子后验子块的组合必须严格互斥。"""
    with pytest.raises(ValueError):
        UnifiedHardwareConfig(
            estimator=estimator,  # type: ignore[arg-type]
            depth_friction_prior=DepthFrictionPriorConfig() if with_depth else None,
            particle_friction=ParticleFrictionConfig() if with_particle else None,
        )


def test_particle_estimator_allows_optional_depth_prior():
    """particle 路线必须带粒子后验，深度先验可选叠加。"""
    UnifiedHardwareConfig(estimator="particle", particle_friction=ParticleFrictionConfig())
    UnifiedHardwareConfig(
        estimator="particle",
        particle_friction=ParticleFrictionConfig(),
        depth_friction_prior=DepthFrictionPriorConfig(),
    )


def test_depth_prior_estimator_requires_depth_block_only():
    """depth_prior 路线必须提供深度先验且不得配置粒子后验。"""
    UnifiedHardwareConfig(estimator="depth_prior", depth_friction_prior=DepthFrictionPriorConfig())


def test_unified_estimator_enum_is_strict():
    """estimator 判别字符串必须三者之一。"""
    with pytest.raises(ValueError, match="classic、depth_prior 或 particle"):
        UnifiedHardwareConfig(estimator="kalman")


@pytest.mark.parametrize("taxels", [0, 10, -1, 3.5, True, False])
def test_min_event_taxels_must_be_integer_within_one_to_nine(taxels):
    """事件摩擦候选门禁必须是 1..9 的整数，布尔值不得冒充整数。"""
    with pytest.raises(ValueError, match="min_event_taxels"):
        UnifiedFrictionConfig(min_event_taxels=taxels)


def test_yaml_rejects_non_integer_min_event_taxels(tmp_path: Path):
    """严格 YAML 同样拒绝非整数的触点门禁。"""
    path = tmp_path / "taxels.yaml"
    path.write_text(
        "reference:\n  unified:\n    friction:\n      min_event_taxels: 3.5\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="min_event_taxels 必须是整数"):
        load_experiment_config(path)


def test_friction_config_validates_priors_and_gains():
    """摩擦先验、安全系数与增益必须是正有限量，安全系数至少为 1。"""
    with pytest.raises(ValueError, match="safety_factor"):
        UnifiedFrictionConfig(safety_factor=0.9)
    with pytest.raises(ValueError, match="left_friction"):
        UnifiedFrictionConfig(left_friction=0.0)
    with pytest.raises(ValueError, match="gap_gain_per_s"):
        UnifiedFrictionConfig(gap_gain_per_s=-1.0)
    with pytest.raises(ValueError, match="filter_tau_s"):
        UnifiedFrictionConfig(filter_tau_s=0.0)
    with pytest.raises(ValueError, match="allow_target_decrease"):
        UnifiedFrictionConfig(allow_target_decrease="yes")  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="friction_expiry_s"):
        UnifiedFrictionConfig(friction_expiry_s=0.0)
    assert UnifiedFrictionConfig(friction_expiry_s=None).friction_expiry_s is None


def test_observer_config_validates_windows_and_flags():
    """观察窗口参数必须为正有限量，两个开关必须是布尔值。"""
    with pytest.raises(ValueError, match="window_s"):
        UnifiedObserverConfig(window_s=0.0)
    with pytest.raises(ValueError, match="confirmation_s"):
        UnifiedObserverConfig(confirmation_s=-0.01)
    with pytest.raises(ValueError, match="min_normal_n"):
        UnifiedObserverConfig(min_normal_n=0.0)
    with pytest.raises(ValueError, match="allow_steady_load_risk"):
        UnifiedObserverConfig(allow_steady_load_risk=1)  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="stable_contact_subset"):
        UnifiedObserverConfig(stable_contact_subset=None)  # type: ignore[arg-type]


def test_timing_defaults_and_ordering_constraints():
    """时序默认值为 250 Hz；控制间隙必须落在单周期与触觉超时之间。"""
    timing = TimingConfig()
    assert timing.control_rate_hz == pytest.approx(250.0)
    assert timing.max_control_gap_s == pytest.approx(0.1)
    assert timing.tactile_timeout_s == pytest.approx(0.2)
    assert timing.tactile_cutoff_hz == pytest.approx(20.0)
    with pytest.raises(ValueError, match="max_control_gap_s 必须大于一个控制周期"):
        TimingConfig(max_control_gap_s=1.0 / 250.0)
    with pytest.raises(ValueError, match="max_control_gap_s 不得大于"):
        TimingConfig(max_control_gap_s=0.3)
    with pytest.raises(ValueError, match="tactile_timeout_s"):
        TimingConfig(tactile_timeout_s=0.0)
    with pytest.raises(ValueError, match="tactile_cutoff_hz"):
        TimingConfig(tactile_cutoff_hz=-1.0)
    with pytest.raises(ValueError, match="有限数值"):
        TimingConfig(control_rate_hz=True)


def test_hardware_home_and_feedback_margin_are_strictly_validated():
    """home 必须位于命令工作范围，反馈余量与容差不得为负。"""
    with pytest.raises(ValueError, match="home_position_rad"):
        HardwareConfig(home_position_rad=-0.01)
    with pytest.raises(ValueError, match="home_position_rad"):
        HardwareConfig(home_position_rad=3.15)
    with pytest.raises(ValueError, match="home_tolerance_rad"):
        HardwareConfig(home_tolerance_rad=-0.01)
    with pytest.raises(ValueError, match="feedback_position_margin_rad"):
        HardwareConfig(feedback_position_margin_rad=-0.01)
    with pytest.raises(ValueError, match="反馈位置安全范围.*超出电机协议位置量程"):
        HardwareConfig(feedback_position_margin_rad=2.0)
    with pytest.raises(ValueError, match="dm_port"):
        HardwareConfig(dm_port="  ")
    with pytest.raises(ValueError, match="tactile_port"):
        HardwareConfig(tactile_port="")


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("mit_kp", 500.01),
        ("mit_kd", 5.01),
        ("velocity_limit_rad_s", 8.01),
        ("torque_limit_nm", 4.01),
        ("return_mit_kp", 500.01),
        ("return_mit_kd", 5.01),
        ("return_torque_limit_nm", 4.01),
    ],
)
def test_controller_mit_fields_cannot_exceed_dm_protocol_ranges(name: str, value: float):
    """实验配置不得让协议饱和改变共享核的安全计算。"""
    with pytest.raises(ValueError, match=rf"controller\.{name}.*协议上限"):
        ControllerConfig(**{name: value})


@pytest.mark.parametrize(
    "name",
    [
        "mit_kp",
        "mit_kd",
        "velocity_limit_rad_s",
        "torque_limit_nm",
        "return_mit_kp",
        "return_mit_kd",
        "return_torque_limit_nm",
    ],
)
@pytest.mark.parametrize("value", [0.0, -1.0, float("nan"), float("inf"), True])
def test_controller_numeric_fields_must_be_positive_finite(name: str, value: float):
    """控制器数值字段必须为正有限量；零、负与非有限值一律非法。"""
    with pytest.raises(ValueError, match=f"controller\\.{name}"):
        ControllerConfig(**{name: value})


@pytest.mark.parametrize(
    "name,value",
    [
        ("max_closing_velocity_m_s", 0.0),
        ("max_opening_velocity_m_s", -0.1),
        ("max_acceleration_m_s2", float("inf")),
    ],
)
def test_admittance_motion_limits_must_be_positive_finite(name: str, value: float) -> None:
    """独立外环运动边界不能以零、负值或非有限数绕过。"""
    with pytest.raises(ValueError):
        AdmittanceConfig(**{name: value})


def test_admittance_stiffness_adaptation_bounds_initial_state():
    """初始导纳质量必须位于调度范围且阻尼必须为正。"""
    adaptive = StiffnessAdmittanceConfig(min_mass_kg=0.5, max_mass_kg=300.0)
    with pytest.raises(ValueError, match="初始导纳质量"):
        AdmittanceConfig(mass_kg=0.2, damping_ns_m=15.0, stiffness_adaptation=adaptive)


def test_safety_ordering_and_positive_bounds():
    """目标上限必须严格低于硬保护线，全部字段必须为正有限量。"""
    with pytest.raises(ValueError, match="max_target_force_n 必须严格小于"):
        SafetyConfig(max_target_force_n=40.0, force_ceiling_n=40.0)
    with pytest.raises(ValueError, match="max_target_force_n"):
        SafetyConfig(max_target_force_n=0.0)
    with pytest.raises(ValueError, match="force_ceiling_n"):
        SafetyConfig(force_ceiling_n=0.0)
    with pytest.raises(ValueError, match="max_force_rate_n_s"):
        SafetyConfig(max_force_rate_n_s=0.0)
    with pytest.raises(ValueError, match="max_force_decrease_rate_n_s"):
        SafetyConfig(max_force_decrease_rate_n_s=-1.0)
    with pytest.raises(ValueError, match="approach_feedforward_force_n"):
        SafetyConfig(approach_feedforward_force_n=0.0)
    with pytest.raises(ValueError, match="max_contact_compression_m"):
        SafetyConfig(max_contact_compression_m=0.0)


def test_reference_bounds_against_contact_and_target():
    """初始目标力必须落在接触阈值与目标上限之间。"""
    with pytest.raises(ValueError, match="不得大于 safety.max_target_force_n"):
        ExperimentConfig(reference=AdaptiveReferenceConfig(initial_force_n=31.0))
    with pytest.raises(ValueError, match="不得小于 lifecycle.contact_on_n"):
        ExperimentConfig(reference=AdaptiveReferenceConfig(initial_force_n=0.1))


def test_reference_duration_accepts_null_and_rejects_invalid_values():
    """显式 null 表示不限时，非法时长一律拒绝。"""
    assert AdaptiveReferenceConfig(duration_s=None).duration_s is None
    for duration in (0.0, -1.0, float("inf"), float("nan"), True):
        with pytest.raises(ValueError):
            AdaptiveReferenceConfig(duration_s=duration)
    with pytest.raises(ValueError, match="initial_force_n"):
        AdaptiveReferenceConfig(initial_force_n=0.0)


def test_unified_without_duration_keeps_hold_until_release():
    """不限时统一任务在 hold 中等待人工释放。"""
    config = ExperimentConfig(reference=_unified_reference())
    assert config.reference.duration_s is None
    assert config.lifecycle.on_finished == "hold"


def _base_stiffness_experiment() -> ExperimentConfig:
    """构造开启刚度预载的最小合法实验。"""
    return ExperimentConfig(
        reference=AdaptiveReferenceConfig(
            initial_force_n=1.0,
            experimental_closed_loop=True,
            unified=UnifiedHardwareConfig(),
        ),
        lifecycle=LifecycleConfig(
            stiffness_preload=StiffnessPreloadConfig(
                min_force_n=1.0,
                probe_force_n=1.0,
                max_force_n=4.0,
                force_ceiling_n=5.0,
                timeout_s=3.0,
            )
        ),
    )


def test_stiffness_preload_valid_configuration_constructs():
    """合法刚度预载配置可离线构造并通过预算校验。"""
    config = _base_stiffness_experiment()
    assert config.lifecycle.stiffness_preload is not None
    config.unified_core_config


def test_stiffness_preload_requires_estimation_and_unified():
    """刚度闭环要求开启估计并使用统一自适应模式。"""
    with pytest.raises(ValueError, match="刚度闭环要求开启估计"):
        ExperimentConfig(
            lifecycle=LifecycleConfig(
                stiffness_preload=StiffnessPreloadConfig(min_force_n=1.0, probe_force_n=1.0)
            )
        )
    with pytest.raises(ValueError, match="刚度闭环要求开启估计"):
        replace(_base_stiffness_experiment(), estimation=EstimationConfig(enabled=False))


def test_stiffness_preload_requires_closed_loop_authorization():
    """刚度闭环同样需要显式 experimental_closed_loop 授权。"""
    base = _base_stiffness_experiment()
    with pytest.raises(ValueError, match="experimental_closed_loop"):
        replace(
            base,
            reference=AdaptiveReferenceConfig(
                initial_force_n=1.0,
                unified=UnifiedHardwareConfig(),
            ),
        )


def test_stiffness_preload_budget_and_bound_validations():
    """预载过力上限、目标范围与时间预算不允许被静默越过。"""
    base = _base_stiffness_experiment()
    with pytest.raises(ValueError, match="预载过力上限"):
        replace(
            base,
            safety=SafetyConfig(max_target_force_n=4.0, force_ceiling_n=4.5),
        )
    with pytest.raises(ValueError, match="刚度预载需要有限的 preload_force_rate_n_s"):
        replace(
            base,
            lifecycle=LifecycleConfig(
                preload_force_rate_n_s=None,
                stiffness_preload=StiffnessPreloadConfig(min_force_n=1.0, probe_force_n=1.0),
            ),
        )
    with pytest.raises(ValueError, match="刚度预载目标必须位于统一调度力范围内"):
        replace(
            base,
            lifecycle=LifecycleConfig(
                stiffness_preload=StiffnessPreloadConfig(
                    min_force_n=0.5,
                    probe_force_n=0.5,
                    max_force_n=4.0,
                )
            ),
        )
    with pytest.raises(ValueError, match="刚度辨识及升力确认预算超过预载超时"):
        replace(
            base,
            lifecycle=LifecycleConfig(
                preload_timeout_s=5.0,
                stiffness_preload=StiffnessPreloadConfig(
                    min_force_n=1.0,
                    probe_force_n=1.0,
                    max_force_n=4.0,
                    timeout_s=4.5,
                ),
            ),
        )


def test_lifecycle_contact_thresholds_and_ratios_are_validated():
    """接触阈值滞回与预载比例的非法值一律拒绝。"""
    with pytest.raises(ValueError, match="contact_off_n"):
        LifecycleConfig(contact_on_n=0.1, contact_off_n=0.2)
    with pytest.raises(ValueError, match="contact_on_stable_s"):
        LifecycleConfig(contact_on_stable_s=0.0)
    with pytest.raises(ValueError, match="contact_off_stable_s"):
        LifecycleConfig(contact_off_stable_s=0.0)
    with pytest.raises(ValueError, match="contact_transition_s"):
        LifecycleConfig(contact_transition_s=0.0)
    with pytest.raises(ValueError, match="preload_stable_time_s"):
        LifecycleConfig(preload_stable_time_s=0.0)
    with pytest.raises(ValueError, match="preload_timeout_s"):
        LifecycleConfig(preload_timeout_s=0.0)


@pytest.mark.parametrize("rate", [0.0, -1.0, float("nan"), float("inf"), True])
def test_preload_ramp_rejects_invalid_rate(rate):
    """预载升力速率必须是正有限数值，None 表示常值目标。"""
    with pytest.raises(ValueError):
        LifecycleConfig(preload_force_rate_n_s=rate)
    assert LifecycleConfig(preload_force_rate_n_s=None).preload_force_rate_n_s is None


@pytest.mark.parametrize("ratio", [0.0, -1.0, 1.01, float("nan"), float("inf"), True])
def test_preload_min_force_ratio_rejects_invalid_values(ratio):
    """预载最低比例必须位于零与一之间。"""
    with pytest.raises(ValueError):
        LifecycleConfig(preload_min_force_ratio=ratio)


@pytest.mark.parametrize("behavior", ["release", "loop"])
def test_on_finished_enum_is_strict(behavior: str):
    """任务结束行为只允许 hold 或 return。"""
    with pytest.raises(ValueError, match="hold 或 return"):
        LifecycleConfig(on_finished=behavior)  # type: ignore[arg-type]
    assert LifecycleConfig(on_finished="return").on_finished == "return"


def test_estimation_only_carries_enabled_switch():
    """估计段只保留 enabled 开关，方法字段已固化为内部常数。"""
    assert EstimationConfig().enabled is True
    with pytest.raises(ValueError, match="enabled 必须是布尔值"):
        EstimationConfig(enabled=1)  # type: ignore[arg-type]


def test_recording_config_validates_stride_and_caps():
    """抽样默认 4；上限参数非法值被拒绝，None 表示显式关闭上限。"""
    assert RecordingConfig().tactile_stride == 4
    assert RecordingConfig().max_duration_s == 600.0
    assert RecordingConfig().max_tactile_mib == 512.0
    assert RecordingConfig(max_duration_s=None).max_duration_s is None
    assert RecordingConfig(max_tactile_mib=None).max_tactile_mib is None
    with pytest.raises(ValueError, match="tactile_stride"):
        RecordingConfig(tactile_stride=0)
    with pytest.raises(ValueError, match="tactile_stride"):
        RecordingConfig(tactile_stride=True)
    with pytest.raises(ValueError, match="max_duration_s"):
        RecordingConfig(max_duration_s=0.0)
    with pytest.raises(ValueError, match="max_tactile_mib"):
        RecordingConfig(max_tactile_mib=-1.0)


def test_yaml_records_recording_section(tmp_path: Path):
    """recording 节可从严格 YAML 加载，未给出的字段使用默认值。"""
    path = tmp_path / "recording.yaml"
    path.write_text(
        "recording:\n  tactile_stride: 8\n  max_duration_s: 300.0\n  max_tactile_mib: null\n",
        encoding="utf-8",
    )
    config = load_experiment_config(path)
    assert config.recording == RecordingConfig(
        tactile_stride=8, max_duration_s=300.0, max_tactile_mib=None
    )


@pytest.mark.parametrize(
    ("section", "snippet", "field"),
    [
        ("metadata", "  description: 旧说明\n", "description"),
        ("metadata", "  tags:\n  - real\n", "tags"),
        ("lifecycle", "  auto_start: true\n", "auto_start"),
        ("lifecycle", "  lost_contact_action: fault\n", "lost_contact_action"),
        ("reference", "  curve:\n    interpolation: linear\n", "curve"),
        ("reference", "  max_force_rate_n_s: 10.0\n", "max_force_rate_n_s"),
        ("controller", "  kind: pid\n", "kind"),
        ("controller", "  pid:\n    mit_kp: 10.0\n", "pid"),
        ("controller", "  adrc: {}\n", "adrc"),
        ("safety", "  approach_feedforward_ratio: 1.0\n", "approach_feedforward_ratio"),
        ("estimation", "  method: window_linear\n", "method"),
    ],
)
def test_yaml_rejects_removed_fields_as_unknown(
    tmp_path: Path, section: str, snippet: str, field: str
):
    """已删除字段一律按未知字段拒绝，不做静默默认。"""
    path = tmp_path / "legacy.yaml"
    path.write_text(f"{section}:\n{snippet}", encoding="utf-8")
    with pytest.raises(ValueError, match=f"未知字段.*{field}"):
        load_experiment_config(path)


@pytest.mark.parametrize(
    ("section", "snippet", "field"),
    [
        ("unified", "    friction_quality_min: 0.3\n", "friction_quality_min"),
        ("unified", "    minimum_event_quality: 0.3\n", "minimum_event_quality"),
        ("observer", "      contact_release_ratio: 0.66\n", "contact_release_ratio"),
        ("observer", "      max_gap_s: 0.1\n", "max_gap_s"),
    ],
)
def test_yaml_rejects_removed_unified_fields_as_unknown(
    tmp_path: Path, section: str, snippet: str, field: str
):
    """unified 子块中固化为常量或换名的旧字段按未知字段拒绝。"""
    path = tmp_path / "legacy_unified.yaml"
    path.write_text(f"reference:\n  unified:\n{snippet}", encoding="utf-8")
    with pytest.raises(ValueError, match=f"未知字段.*{field}"):
        load_experiment_config(path)


@pytest.mark.parametrize("section", ["terminal", "output"])
def test_yaml_rejects_removed_root_sections(tmp_path: Path, section: str):
    """根节点出现 terminal／output 段必须报未知字段。"""
    path = tmp_path / f"{section}.yaml"
    path.write_text(f"{section}:\n  mode: auto\n", encoding="utf-8")
    with pytest.raises(ValueError, match="配置根节点包含未知字段"):
        load_experiment_config(path)


def test_yaml_root_rejects_multiple_unknown_sections(tmp_path: Path):
    """根节点同时出现两个未知段时按排序列出。"""
    path = tmp_path / "roots.yaml"
    path.write_text("terminal:\n  mode: auto\noutput:\n  root: x\n", encoding="utf-8")
    with pytest.raises(ValueError, match="配置根节点包含未知字段：output, terminal"):
        load_experiment_config(path)


def test_yaml_strict_loading_rejects_unknown_and_bad_types(tmp_path: Path):
    """严格 YAML 拒绝未知字段、布尔冒充数值与无效枚举。"""
    document = """
metadata:
  task_name: grasp-demo
  object_name: cube
timing:
  control_rate_hz: 100.0
  max_control_gap_s: 0.15
  tactile_timeout_s: 0.2
reference:
  initial_force_n: 0.6
  duration_s: 5.0
  experimental_closed_loop: true
  unified:
    estimator: classic
"""
    path = tmp_path / "config.yaml"
    path.write_text(document, encoding="utf-8")
    config = load_experiment_config(path)
    assert config.metadata.task_name == "grasp-demo"
    assert config.reference.initial_force_n == pytest.approx(0.6)
    assert config.reference.duration_s == pytest.approx(5.0)
    assert config.reference.unified is not None
    assert config.reference.unified.estimator == "classic"

    unknown = tmp_path / "unknown.yaml"
    unknown.write_text(document.replace("timing:", "timing:\n  mystery: 1.0\n"), encoding="utf-8")
    with pytest.raises(ValueError, match="未知字段"):
        load_experiment_config(unknown)

    boolean = tmp_path / "boolean.yaml"
    boolean.write_text(
        document.replace("control_rate_hz: 100.0", "control_rate_hz: true"), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="有限数值"):
        load_experiment_config(boolean)

    enum = tmp_path / "enum.yaml"
    enum.write_text(document.replace("estimator: classic", "estimator: kalman"), encoding="utf-8")
    with pytest.raises(ValueError, match="之一"):
        load_experiment_config(enum)


def test_yaml_metadata_rejects_empty_names(tmp_path: Path):
    """metadata 只保留任务与物体名，且必须是非空字符串。"""
    path = tmp_path / "metadata.yaml"
    path.write_text('metadata:\n  task_name: "  "\n', encoding="utf-8")
    with pytest.raises(ValueError, match="task_name"):
        load_experiment_config(path)


def test_yaml_hardware_ports_must_be_non_empty_strings(tmp_path: Path):
    """设备端口在 YAML 层同样拒绝空串与非字符串。"""
    path = tmp_path / "ports.yaml"
    path.write_text('hardware:\n  dm_port: ""\n', encoding="utf-8")
    with pytest.raises(ValueError, match="dm_port"):
        load_experiment_config(path)


def test_sanitize_directory_component_blocks_traversal():
    """目录组件清洗阻止路径分隔符、.. 与空名。"""
    assert sanitize_directory_component("倒水-实验 2") == "倒水-实验_2"
    with pytest.raises(ValueError):
        sanitize_directory_component("a/b")
    with pytest.raises(ValueError):
        sanitize_directory_component("..")
    with pytest.raises(ValueError):
        sanitize_directory_component("  ")
    with pytest.raises(ValueError):
        sanitize_directory_component("a\\b")


def test_stiffness_preload_none_remains_valid_without_closed_loop():
    """未启用刚度预载与导纳调度时不需要统一闭环授权。"""
    config = ExperimentConfig()
    assert config.lifecycle.stiffness_preload is None
    assert config.controller.admittance.stiffness_adaptation is None
    assert not config.unified_adaptive_enabled
