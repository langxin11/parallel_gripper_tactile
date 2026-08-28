"""不带子进程或旧版脚本导入的 CLI 集成测试。"""

from pathlib import Path

from typer.testing import CliRunner

from parallel_gripper_tactile.cli import app


ROOT = Path(__file__).resolve().parents[1]
RUNNER = CliRunner()


def test_root_and_subcommand_help_are_available() -> None:
    """规范 CLI 暴露所有公开的命令组。"""
    result = RUNNER.invoke(app, ["--help"])

    assert result.exit_code == 0
    for command in ("validate", "assets", "run", "compare", "view", "runs"):
        assert command in result.output
    assert RUNNER.invoke(app, ["run", "grasp", "--help"]).exit_code == 0
    assert RUNNER.invoke(app, ["run", "force-track", "--help"]).exit_code == 0
    assert RUNNER.invoke(app, ["compare", "tactile", "--help"]).exit_code == 0


def test_validate_profile_and_invalid_yaml_exit_codes(tmp_path: Path) -> None:
    """配置校验成功，配置错误时使用退出码二。"""
    profile = ROOT / "configs" / "robotiq_2f85.yaml"
    assert RUNNER.invoke(app, ["validate", str(profile)]).exit_code == 0

    invalid = tmp_path / "invalid.yaml"
    invalid.write_text("schema_version: 9\n", encoding="utf-8")
    result = RUNNER.invoke(app, ["validate", str(invalid)])
    assert result.exit_code == 2
    assert "Profile validation failed" in result.output


def test_runs_list_and_clean_are_safe_when_output_root_is_absent(tmp_path: Path) -> None:
    """运行检查与清理在全新工作区上仍保持 dry-run 安全。"""
    output_root = tmp_path / "missing"
    assert RUNNER.invoke(app, ["runs", "list", "--output-root", str(output_root)]).exit_code == 0
    result = RUNNER.invoke(app, ["runs", "clean", "--all", "--output-root", str(output_root)])
    assert result.exit_code == 0
    assert "Would remove 0" in result.output
