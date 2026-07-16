"""验证 Robotiq 2F-85 taxel 派生资产的结构。"""

from __future__ import annotations

import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def _load_generator():
    """从脚本文件加载生成模块，避免要求安装为 Python 包。"""
    path = Path(__file__).resolve().parents[1] / "scripts/generate_taxels_xml.py"
    spec = importlib.util.spec_from_file_location("generate_taxels_xml", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_taxel_tree_creates_two_3x3_taxel_grids() -> None:
    """派生模型应在两个指尖各创建 3×3 taxel 网格。"""
    generator = _load_generator()
    root = generator.build_taxel_tree(generator.DEFAULT_BASE_XML).getroot()
    sites = root.findall(".//site")
    assert len([site for site in sites if "_taxel_site_" in site.get("name", "")]) == 18
    sensors = root.findall("./sensor/force")
    assert len([sensor for sensor in sensors if "_taxel_force_" in sensor.get("name", "")]) == 18


def test_generated_xml_is_well_formed(tmp_path: Path) -> None:
    """命令行生成的 XML 应能被标准库重新解析。"""
    generator = _load_generator()
    output = tmp_path / "2f85_taxels.xml"
    tree = generator.build_taxel_tree(generator.DEFAULT_BASE_XML)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    assert ET.parse(output).getroot().get("model") == "robotiq_2f85_taxels"


def test_taxel_report_sensor_names_follow_row_major_order() -> None:
    """读数报告脚本应按左右各自从 00 到 22 的行优先顺序映射传感器。"""
    report_path = Path(__file__).resolve().parents[1] / "scripts/report_taxels.py"
    spec = importlib.util.spec_from_file_location("report_taxels", report_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    assert module.TAXEL_SENSOR_NAMES == (
        "left_taxel_force_00",
        "left_taxel_force_01",
        "left_taxel_force_02",
        "left_taxel_force_10",
        "left_taxel_force_11",
        "left_taxel_force_12",
        "left_taxel_force_20",
        "left_taxel_force_21",
        "left_taxel_force_22",
        "right_taxel_force_00",
        "right_taxel_force_01",
        "right_taxel_force_02",
        "right_taxel_force_10",
        "right_taxel_force_11",
        "right_taxel_force_12",
        "right_taxel_force_20",
        "right_taxel_force_21",
        "right_taxel_force_22",
    )


def test_runtime_grasp_scene_attaches_cube_and_preserves_world_setup() -> None:
    """运行时场景应挂接自由方块，并保留重力、薄板和程序化天空盒。"""
    scene_path = Path(__file__).resolve().parents[1] / "scripts/grasp_scene.py"
    spec = importlib.util.spec_from_file_location("grasp_scene", scene_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    root = ET.parse(module.GRASP_WORLD_XML).getroot()
    assert root.find(".//geom[@name='ground']") is not None
    support_plate = root.find(".//geom[@name='target_cube_support_plate']")
    assert support_plate is not None
    assert support_plate.get("type") == "box"
    assert root.find("option").get("gravity") == "0 0 -9.81"
    starfield = root.find("./asset/texture[@name='starfield_skybox']")
    assert starfield is not None
    assert starfield.get("type") == "skybox"
    assert starfield.get("builtin") == "gradient"
    assert starfield.get("mark") == "random"
    assert starfield.get("random") == "0.008"
    cube_root = ET.parse(module.TARGET_CUBE_XML).getroot()
    assert cube_root.find(".//body[@name='target_cube']") is not None
    assert cube_root.find(".//freejoint[@name='target_cube_free_joint']") is not None


def test_runtime_grasp_scene_compiles_taxel_and_touch_grid_assets() -> None:
    """attach 后应保留实体前缀，并支持默认及低分辨率触觉资产。"""
    import mujoco

    scene_path = Path(__file__).resolve().parents[1] / "scripts/grasp_scene.py"
    spec = importlib.util.spec_from_file_location("grasp_scene_compilation", scene_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    asset_dir = Path(__file__).resolve().parents[1] / "assets/robotiq_2f85"

    taxel_model = module.build_grasp_spec(asset_dir / "2f85_taxels.xml").compile()
    assert mujoco.mj_name2id(
        taxel_model, mujoco.mjtObj.mjOBJ_SENSOR, "gripper/left_taxel_force_00"
    ) >= 0
    assert mujoco.mj_name2id(taxel_model, mujoco.mjtObj.mjOBJ_BODY, "cube/target_cube") >= 0
    assert mujoco.mj_name2id(taxel_model, mujoco.mjtObj.mjOBJ_KEY, "closed") >= 0

    grid_model = module.build_grasp_spec(asset_dir / "2f85_touch_grid_3x3.xml").compile()
    assert mujoco.mj_name2id(grid_model, mujoco.mjtObj.mjOBJ_SENSOR, "gripper/touch_left") >= 0


def test_touch_grid_model_creates_two_32_by_32_collision_pads() -> None:
    """touch_grid 版本应为左右指腹各生成 1024 个碰撞单元和一个插件传感器。"""
    script_path = Path(__file__).resolve().parents[1] / "scripts/generate_touch_grid_xml.py"
    spec = importlib.util.spec_from_file_location("generate_touch_grid_xml", script_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    root = module.build_touch_grid_tree(module.DEFAULT_BASE_XML).getroot()
    cells = root.findall(".//geom[@name]")
    assert len([cell for cell in cells if "_touch_cell_" in cell.get("name", "")]) == 2048
    assert len(root.findall("./sensor/plugin")) == 2


def test_touch_grid_model_supports_three_by_three_resolution() -> None:
    """生成器应支持低分辨率的 3×3 三通道触觉网格。"""
    script_path = Path(__file__).resolve().parents[1] / "scripts/generate_touch_grid_xml.py"
    spec = importlib.util.spec_from_file_location("generate_touch_grid_3x3", script_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    root = module.build_touch_grid_tree(module.DEFAULT_BASE_XML, rows=3, cols=3).getroot()
    cells = root.findall(".//geom[@name]")
    assert len([cell for cell in cells if "_touch_cell_" in cell.get("name", "")]) == 18
    assert [config.get("value") for config in root.findall("./sensor/plugin/config[@key='size']")] == [
        "3 3",
        "3 3",
    ]
