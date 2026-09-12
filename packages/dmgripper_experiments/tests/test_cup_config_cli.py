"""验证倒水配置与不接触设备的命令行路径。"""

import json
from pathlib import Path

import pytest

from dmgripper_experiments.cup_cli import run
from dmgripper_experiments.cup_config import CupConfig, load_cup_config


@pytest.mark.parametrize("controller", ["admittance", "pid", "adrc"])
def test_three_controllers_are_valid(controller: str) -> None:
    """三种已登记控制器可以构造。"""
    assert CupConfig(controller=controller).controller == controller


def test_dry_run_does_not_import_runtime(capsys: pytest.CaptureFixture[str]) -> None:
    """默认命令只打印计划。"""
    assert run(["--controller", "pid", "--control.target-force-n", "0.6"]) == 0
    record = json.loads(capsys.readouterr().out)
    assert record["mode"] == "dry-run"
    assert record["config"]["controller"] == "pid"
    assert record["config"]["control"]["target_force_n"] == 0.6


def test_yaml_is_overridden_by_tyro(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """YAML 先覆盖默认值，Tyro 再覆盖 YAML。"""
    path = tmp_path / "cup.yaml"
    path.write_text("controller: adrc\ncontrol:\n  target_force_n: 0.4\n", encoding="utf-8")
    assert run(["--config", str(path), "--control.target-force-n", "0.6"]) == 0
    record = json.loads(capsys.readouterr().out)
    assert record["config"]["controller"] == "adrc"
    assert record["config"]["control"]["target_force_n"] == 0.6


@pytest.mark.parametrize(
    "contents",
    ["execute: true\n", "control:\n  target_force_n: true\n", "grip:\n  unknown: 1\n"],
)
def test_yaml_rejects_operations_and_invalid_types(tmp_path: Path, contents: str) -> None:
    """YAML 只能表达已登记的强类型配置。"""
    path = tmp_path / "invalid.yaml"
    path.write_text(contents, encoding="utf-8")
    with pytest.raises(ValueError):
        load_cup_config(path)


def test_cross_field_constraints_are_checked(tmp_path: Path) -> None:
    """目标力必须满足倒水抓握的安全关系。"""
    path = tmp_path / "invalid.yaml"
    path.write_text("control:\n  target_force_n: 0.1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="contact_on"):
        load_cup_config(path)
