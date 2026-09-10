"""针对通用触觉读取器层的契约测试。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import pytest

from parallel_gripper_tactile.tactile import (
    ContactGeomTactileReader,
    ForceSensorTactileReader,
    TactileFrame,
    TouchGridTactileReader,
    create_tactile_reader,
)

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True, slots=True)
class _Layout:
    """独立于旧版加载器的小型配置形状对象。"""

    mode: str
    rows: int
    cols: int
    left_prefix: str
    right_prefix: str


def _sensor_address(model: mujoco.MjModel, name: str) -> int:
    """查找传感器的首个输出地址以用于测试设置。"""
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    assert sensor_id >= 0
    return int(model.sensor_adr[sensor_id])


def test_tactile_frame_enforces_common_shape_and_immutable_storage() -> None:
    """每个读取器输出都使用有限且等大小的 xyz 力网格。"""
    source = np.zeros((3, 2, 3))
    frame = TactileFrame(source, source)

    source[0, 0, 0] = 7.0
    assert frame.left[0, 0, 0] == 0.0
    assert not frame.left.flags.writeable
    with pytest.raises(ValueError, match="same shape"):
        TactileFrame(np.zeros((3, 2, 3)), np.zeros((3, 3, 2)))
    with pytest.raises(ValueError, match="finite"):
        TactileFrame(np.full((3, 1, 1), np.nan), np.zeros((3, 1, 1)))


def test_force_sensor_reader_normalizes_surface_load_sign() -> None:
    """触觉元反应传感器转换为局部 xyz 表面载荷，并带正压缩。"""
    model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/grippers/robotiq_2f85/2f85_taxels.xml"))
    data = mujoco.MjData(model)
    layout = _Layout("force_sensor", 3, 3, "left_taxel_force_", "right_taxel_force_")
    reader = ForceSensorTactileReader(model, layout)
    left_address = _sensor_address(model, "left_taxel_force_12")
    right_address = _sensor_address(model, "right_taxel_force_12")
    data.sensordata[left_address : left_address + 3] = (1.0, -2.0, -3.0)
    data.sensordata[right_address : right_address + 3] = (-4.0, 5.0, -6.0)

    frame = reader.read(data)

    assert frame.left.shape == (3, 3, 3)
    np.testing.assert_allclose(frame.left[:, 1, 2], (-1.0, 2.0, 3.0))
    np.testing.assert_allclose(frame.right[:, 1, 2], (4.0, -5.0, 6.0))


def test_touch_grid_reader_reorders_plugin_zxy_to_common_xyz_contract() -> None:
    """插件 zxy 存储顺序映射为公开的 xyz 网格顺序。"""
    model = mujoco.MjModel.from_xml_path(
        str(ROOT / "assets/grippers/robotiq_2f85/2f85_touch_grid_3x3.xml")
    )
    data = mujoco.MjData(model)
    layout = _Layout("touch_grid", 3, 3, "touch_left", "touch_right")
    reader = TouchGridTactileReader(model, layout)
    left_address = _sensor_address(model, "touch_left")
    right_address = _sensor_address(model, "touch_right")
    plugin_values = np.arange(27, dtype=np.float64).reshape(3, 3, 3)
    data.sensordata[left_address : left_address + 27] = plugin_values.ravel()
    data.sensordata[right_address : right_address + 27] = (100 + plugin_values).ravel()

    frame = reader.read(data)

    np.testing.assert_array_equal(frame.left, plugin_values[[1, 2, 0]])
    np.testing.assert_array_equal(frame.right, (100 + plugin_values)[[1, 2, 0]])


def test_factory_selects_each_supported_reader_and_resolves_namespaces() -> None:
    """工厂支持所有公开后端，而无需导入配置代码。"""
    force_model = mujoco.MjModel.from_xml_path(
        str(ROOT / "assets/grippers/robotiq_2f85/2f85_taxels.xml")
    )
    force_layout = _Layout("force_sensor", 3, 3, "left_taxel_force_", "right_taxel_force_")
    assert isinstance(create_tactile_reader(force_model, force_layout), ForceSensorTactileReader)

    contact_model = mujoco.MjModel.from_xml_path(
        str(ROOT / "assets/grippers/dm_gripper/parallel_gripper_prepared.xml")
    )
    contact_layout = _Layout("contact_geom", 3, 3, "left_taxel_geom_", "right_taxel_geom_")
    reader = create_tactile_reader(contact_model, contact_layout)
    assert isinstance(reader, ContactGeomTactileReader)
    assert reader.shape == (3, 3, 3)

    with pytest.raises(ValueError, match="unsupported tactile mode"):
        create_tactile_reader(force_model, _Layout("invalid", 3, 3, "left", "right"))
