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
