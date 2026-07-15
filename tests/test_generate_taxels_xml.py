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


def test_cube_grasp_scene_places_cube_between_taxel_pads() -> None:
    """闭合抓取场景应包含两个指尖中间的自由正方体和闭合 keyframe。"""
    scene_path = Path(__file__).resolve().parents[1] / "scripts/generate_cube_grasp_scene.py"
    spec = importlib.util.spec_from_file_location("generate_cube_grasp_scene", scene_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    root = module.build_cube_grasp_tree(module.DEFAULT_GRIPPER_XML).getroot()
    assert root.find(".//body[@name='target_cube']") is not None
    assert root.find(".//freejoint[@name='target_cube_free_joint']") is not None
    assert root.find(".//key[@name='closed']").get("ctrl") == "220"


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
