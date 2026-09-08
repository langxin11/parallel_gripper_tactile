"""验证 YAML profile 加载、编译契约和行优先命名约定。"""

from pathlib import Path

import pytest
from pydantic import ValidationError

from parallel_gripper_tactile.profiles import ProfileLoadError, TouchGridTactileLayout, load_profile
from parallel_gripper_tactile.validation import validate_profile


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "profile_name, actuator, channels",
    [
        ("robotiq_2f85.yaml", "fingers_actuator", 18),
        ("robotiq_2f85_box.yaml", "fingers_actuator", 18),
        ("custom_parallel_gripper.yaml", "gripper_drive", 18),
    ],
)
def test_profiles_compile_and_satisfy_contract(
    profile_name: str, actuator: str, channels: int
) -> None:
    """两个 profile 均可编译且满足触觉通道契约。"""
    profile = load_profile(ROOT / "configs" / profile_name)
    report = validate_profile(profile)
    assert report.actuator == actuator
    assert report.tactile_channels == channels


def test_tactile_names_are_row_major() -> None:
    """taxel 命名按行优先（00..22）排列。"""
    profile = load_profile(ROOT / "configs" / "custom_parallel_gripper.yaml")
    assert profile.tactile.names("left") == (
        "left_taxel_geom_00",
        "left_taxel_geom_01",
        "left_taxel_geom_02",
        "left_taxel_geom_10",
        "left_taxel_geom_11",
        "left_taxel_geom_12",
        "left_taxel_geom_20",
        "left_taxel_geom_21",
        "left_taxel_geom_22",
    )


def test_custom_profile_defines_bounded_mit_torque_control() -> None:
    """自研夹爪默认使用保留高度差的稳定碰撞代理与有界 MIT 控制。"""
    profile = load_profile(ROOT / "configs" / "custom_parallel_gripper.yaml")

    assert profile.model_path.name == "parallel_gripper_height_sphere_collision.xml"
    assert profile.control_mode == "mit_torque"
    assert profile.mit is not None
    assert profile.mit.p_min <= profile.open_control < profile.closed_control <= profile.mit.p_max
    assert profile.mit.p_max <= 1.7
    assert profile.mit.v_max <= 8.0
    assert profile.mit.t_max <= 4.0
    assert profile.normal_force is not None
    assert profile.normal_force.target_n == 8.0
    assert profile.normal_force.release_threshold_n < profile.normal_force.contact_threshold_n
    assert profile.normal_force.filter_cutoff_hz == 20.0
    assert profile.normal_force.sensor_taxel_normal_noise_std_n == pytest.approx(
        (0.0066666667, 0.0133333333)
    )
    assert profile.normal_force.sensor_taxel_shear_noise_std_n == pytest.approx(
        (0.0033333333, 0.01)
    )
    assert profile.normal_force.sensor_noise_seed == 20260814
    assert profile.normal_force.geometry is not None
    assert profile.normal_force.geometry.crank_radius_m == pytest.approx(0.03)
    assert profile.normal_force.geometry.link_length_m == pytest.approx(0.04)
    assert profile.normal_force.geometry.offset_m == pytest.approx(0.03 / 2**0.5)
    assert profile.normal_force.stiffness is not None
    assert profile.normal_force.stiffness.method == "window_linear"
    assert profile.normal_force.stiffness.window_size == 25
    assert profile.normal_force.stiffness.min_samples == 8
    # The force loop is expressed in mean single-side force.  These values are
    # the corresponding conversion of the former total-force-tuned baseline.
    assert profile.normal_force.kp == pytest.approx(0.016)
    assert profile.normal_force.ki == pytest.approx(0.2)
    assert profile.normal_force.stiffness.initial_n_per_m == pytest.approx(3000.0)
    assert profile.normal_force.stiffness.min_n_per_m == pytest.approx(250.0)
    assert profile.normal_force.stiffness.max_n_per_m == pytest.approx(25000.0)
    assert profile.normal_force.stiffness.min_delta_force_n == pytest.approx(0.025)
    assert profile.normal_force.stiffness.position_feedforward_gain == pytest.approx(0.25)
    assert profile.normal_force.stiffness.torque_feedforward_gain == pytest.approx(1.0)
    assert profile.normal_force.stiffness.position_limit_enabled is False
    assert profile.normal_force.stiffness.position_limit_force_rate_n_s == pytest.approx(10.0)
    assert profile.normal_force.stiffness.position_limit_stiffness_safety_factor == pytest.approx(
        1.0
    )


def test_stiffness_profile_validates_window_method_requirements() -> None:
    """窗口拟合方法必须配置足够且不超过窗口大小的样本数。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    assert profile.normal_force is not None
    assert profile.normal_force.stiffness is not None

    values = profile.normal_force.stiffness.model_dump()
    with pytest.raises(ValidationError, match="enough samples"):
        type(profile.normal_force.stiffness).model_validate(
            values | {"method": "window_quadratic", "min_samples": 2}
        )

    with pytest.raises(ValidationError, match="must not exceed window_size"):
        type(profile.normal_force.stiffness).model_validate(
            values | {"window_size": 3, "min_samples": 4}
        )

    with pytest.raises(ValidationError, match="mutually exclusive"):
        type(profile.normal_force.stiffness).model_validate(
            values | {"position_limit_enabled": True}
        )


def test_touch_grid_profile_reads_dimensions_from_plugin_configuration() -> None:
    """touch_grid profile 不在 YAML 中重复网格尺寸，而是从 MJCF 插件读取。"""
    profile = load_profile(ROOT / "configs" / "robotiq_2f85_touch_grid.yaml")

    assert isinstance(profile.tactile, TouchGridTactileLayout)
    assert (profile.tactile.rows, profile.tactile.cols) == (3, 3)
    assert profile.tactile.names("left") == ("touch_left",)


def test_relative_model_path_is_resolved_from_profile_file(tmp_path: Path) -> None:
    """模型路径以 profile 所在目录为基准，而非根据仓库结构猜测。"""
    model = tmp_path / "model.xml"
    model.write_text("<mujoco model='minimal'/>", encoding="utf-8")
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(
        """schema_version: 1
name: temporary
model: {path: model.xml}
control: {mode: position, actuator: drive, open: 0, closed: 1}
mount: {pos: [0, 0, 0], quat: [1, 0, 0, 0]}
tactile:
  mode: force_sensor
  rows: 1
  cols: 1
  left_prefix: left_
  right_prefix: right_
""",
        encoding="utf-8",
    )

    assert load_profile(profile_path).model_path == model.resolve()


def test_profile_rejects_unknown_fields_and_illegal_ranges(tmp_path: Path) -> None:
    """Pydantic 拒绝拼错字段和不合法控制范围。"""
    profile_path = tmp_path / "invalid.yaml"
    profile_path.write_text(
        """schema_version: 1
name: invalid
model: {path: missing.xml}
control:
  mode: mit_torque
  actuator: drive
  open: 2
  closed: 0
  mit: {p_min: 0, p_max: 1, v_max: 1, t_max: 1, kp: 0, kd: 0}
mount: {pos: [0, 0, 0], quat: [1, 0, 0, 0]}
tactile: {mode: force_sensor, rows: 1, cols: 1, left_prefix: left_, right_prefix: right_, typo: 1}
""",
        encoding="utf-8",
    )

    with pytest.raises(ValidationError) as error:
        load_profile(profile_path)
    assert "control.mit_torque" in str(error.value)
    assert "tactile.force_sensor.typo" in str(error.value)


@pytest.mark.parametrize(
    "name, contents, error",
    [
        ("empty.yaml", "", "profile is empty"),
        ("not-a-map.yaml", "- one\n- two\n", "root must be a mapping"),
        ("many.yaml", "{}\n---\n{}\n", "exactly one YAML document"),
    ],
)
def test_profile_rejects_invalid_yaml_document_shapes(
    tmp_path: Path, name: str, contents: str, error: str
) -> None:
    """空 YAML、非 mapping 根节点和多文档输入都有清晰错误。"""
    profile_path = tmp_path / name
    profile_path.write_text(contents, encoding="utf-8")

    with pytest.raises(ProfileLoadError, match=error):
        load_profile(profile_path)


def test_toml_profiles_are_rejected() -> None:
    """不保留 TOML loader，避免新旧配置格式混用。"""
    with pytest.raises(ProfileLoadError, match="TOML is unsupported"):
        load_profile(ROOT / "configs" / "custom_parallel_gripper.toml")
