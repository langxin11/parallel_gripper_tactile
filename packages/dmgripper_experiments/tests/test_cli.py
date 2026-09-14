"""新 CLI 的 dry-run 契约：不导入运行时、不创建正式结果目录。"""

from __future__ import annotations

import io
import json
import os
import sys
import time
from pathlib import Path

import pytest


def _wait_for_action(action_source, timeout_s: float = 1.0) -> str:
    """在短时限内等待后台输入线程交付一条命令。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        action = action_source()
        if action is not None:
            return action
        time.sleep(0.005)
    raise AssertionError("输入线程未在时限内交付命令")


def test_action_source_normalizes_lines_and_reports_eof() -> None:
    """交互输入忽略空行、折叠大小写，并在读完后报告关闭。"""
    from dmgripper_experiments import cli

    action_source = cli._action_source(io.StringIO("  s  \n\nStatus\nr\n"))
    assert _wait_for_action(action_source) == "start"
    assert _wait_for_action(action_source) == "status"
    assert _wait_for_action(action_source) == "release"
    with pytest.raises(RuntimeError, match="交互输入已关闭"):
        _wait_for_action(action_source)


def test_action_source_propagates_reader_failure() -> None:
    """后台读取异常必须传回运行线程，不能静默变成永久无输入。"""
    from dmgripper_experiments import cli

    class BrokenInput:
        def __iter__(self):
            raise OSError("模拟终端读取失败")

    action_source = cli._action_source(BrokenInput())
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        try:
            action_source()
        except RuntimeError as error:
            assert "交互输入读取失败" in str(error)
            assert "模拟终端读取失败" in str(error)
            break
        time.sleep(0.005)
    else:
        raise AssertionError("输入线程异常未传回运行线程")


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="需要 POSIX 伪终端")
def test_action_source_receives_start_from_pseudo_terminal() -> None:
    """真实行规程下输入 start 并按 Enter 后能被后台线程收到。"""
    from dmgripper_experiments import cli

    master_fd, slave_fd = os.openpty()
    input_stream = os.fdopen(os.dup(slave_fd), "r", encoding="utf-8", buffering=1)
    try:
        action_source = cli._action_source(input_stream)
        os.write(master_fd, b" START \n")
        assert _wait_for_action(action_source) == "start"
    finally:
        os.close(master_fd)
        os.close(slave_fd)
        input_stream.close()


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="需要 POSIX 伪终端")
def test_rich_action_source_reports_edits_and_restores_terminal() -> None:
    """Rich 输入逐字符更新固定输入区，并在结束后恢复终端。"""
    import termios

    from dmgripper_experiments import cli

    master_fd, slave_fd = os.openpty()
    input_stream = os.fdopen(os.dup(slave_fd), "r", encoding="utf-8", buffering=1)
    original = termios.tcgetattr(input_stream.fileno())
    edits: list[str] = []
    action_source = cli._action_source(input_stream, on_edit=edits.append)
    try:
        os.write(master_fd, b"releasx\x7fe\n")
        assert _wait_for_action(action_source) == "release"
        assert "releasx" in edits
        assert "release" in edits
    finally:
        action_source.close()
        restored = termios.tcgetattr(input_stream.fileno())
        os.close(master_fd)
        os.close(slave_fd)
        input_stream.close()
    assert restored == original


def test_dry_run_outputs_plan_without_runtime(capsys: pytest.CaptureFixture[str]) -> None:
    """dry-run 输出 JSON 计划且本次调用不导入 runtime 模块。"""
    from dmgripper_experiments import cli

    modules_before = set(sys.modules)
    exit_code = cli.run(["--controller.kind", "pid"])
    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert payload["mode"] == "dry-run"
    newly_imported = set(sys.modules) - modules_before
    assert not any(name.startswith("dmgripper_experiments.runtime") for name in newly_imported)


def test_dry_run_with_yaml_config(capsys: pytest.CaptureFixture[str], tmp_path: Path) -> None:
    """--config 加载 YAML 后仍保持 dry-run。"""
    document = """
reference:
  adaptive:
    initial_force_n: 0.6
    duration_s: 5.0
"""
    path = tmp_path / "adaptive.yaml"
    path.write_text(document, encoding="utf-8")
    from dmgripper_experiments import cli

    modules_before = set(sys.modules)
    assert cli.run(["--config", str(path)]) == 0
    payload = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert payload["config"]["reference"]["adaptive"]["initial_force_n"] == pytest.approx(0.6)
    newly_imported = set(sys.modules) - modules_before
    assert not any(name.startswith("dmgripper_experiments.runtime") for name in newly_imported)


def test_invalid_config_reports_error(capsys: pytest.CaptureFixture[str], tmp_path: Path) -> None:
    """非法 YAML 以非零码退出并输出错误，不进入执行。"""
    from dmgripper_experiments import cli

    broken = tmp_path / "broken.yaml"
    broken.write_text("reference:\n  curve:\n    interpolation: cubic\n", encoding="utf-8")
    modules_before = set(sys.modules)
    assert cli.run(["--config", str(broken)]) == 1
    assert "实验失败" in capsys.readouterr().err
    newly_imported = set(sys.modules) - modules_before
    assert not any(name.startswith("dmgripper_experiments.runtime") for name in newly_imported)


def test_execute_without_terminal_requires_unattended_combination(
    capsys: pytest.CaptureFixture[str],
) -> None:
    """非交互 --execute 且缺少无人值守组合时直接报错，不打开设备。"""
    from dmgripper_experiments import cli

    modules_before = set(sys.modules)
    assert cli.run(["--execute"]) == 1
    assert "auto_start" in capsys.readouterr().err
    newly_imported = set(sys.modules) - modules_before
    assert not any(name.startswith("dmgripper_experiments.runtime") for name in newly_imported)


def test_unattended_execute_does_not_start_stdin_reader(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """无人值守组合不读取已关闭的标准输入，以免 EOF 抢在 auto_start 前失败。"""
    from dmgripper_experiments import cli, runtime

    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)

    def forbidden_action_source():
        raise AssertionError("非交互执行不应启动标准输入线程")

    def fake_run_experiment(*_args, action_source, **_kwargs):
        assert action_source() is None
        return {"status": "completed", "final_phase": "completed"}

    monkeypatch.setattr(cli, "_action_source", forbidden_action_source)
    monkeypatch.setattr(runtime, "run_experiment", fake_run_experiment)
    assert (
        cli.run(
            [
                "--execute",
                "--lifecycle.auto-start",
                "--lifecycle.on-finished",
                "return",
                "--terminal.mode",
                "plain",
                "--output",
                str(tmp_path),
            ]
        )
        == 0
    )
    assert '"event": "complete"' in capsys.readouterr().out


def test_plain_execute_prints_preflight_warning(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """纯文本执行模式把使能前告警作为 JSON 行直接输出。"""
    from dmgripper_experiments import cli, runtime

    monkeypatch.setattr(sys.stdin, "isatty", lambda: True)
    monkeypatch.setattr(cli, "_action_source", lambda **_kwargs: lambda: None)

    def fake_run_experiment(*_args, event_sink, **_kwargs):
        assert event_sink is not None
        event_sink(
            {
                "event": "warning",
                "code": "zero_force_peak",
                "message": "使能前零力均值验证通过，但检测到峰值",
            }
        )
        return {"status": "completed", "final_phase": "completed"}

    monkeypatch.setattr(runtime, "run_experiment", fake_run_experiment)
    assert (
        cli.run(
            [
                "--execute",
                "--terminal.mode",
                "plain",
                "--output",
                str(tmp_path),
            ]
        )
        == 0
    )
    payloads = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert any(payload.get("code") == "zero_force_peak" for payload in payloads)


def test_rich_execute_prints_final_safety_summary(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Rich 面板停止后输出状态、失能确认和运行目录的稳定摘要。"""
    from dmgripper_experiments import cli, runtime

    monkeypatch.setattr(sys.stdin, "isatty", lambda: True)
    monkeypatch.setattr(cli, "_action_source", lambda **_kwargs: lambda: None)

    def fake_run_experiment(*_args, **_kwargs):
        return {
            "status": "completed",
            "final_phase": "completed",
            "disable_confirmed": True,
            "output_directory": str(tmp_path / "run"),
        }

    monkeypatch.setattr(runtime, "run_experiment", fake_run_experiment)
    assert (
        cli.run(
            [
                "--execute",
                "--terminal.mode",
                "rich",
                "--output",
                str(tmp_path),
            ]
        )
        == 0
    )
    output = capsys.readouterr().out
    assert "状态=completed" in output
    assert "电机失能=已确认" in output
    assert "运行目录=" in output


def test_keyboard_interrupt_prints_cleanup_warning_and_run_directory(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """中断伴随清理失败时，终端必须醒目显示附加警告和记录目录。"""
    from dmgripper_experiments import cli, runtime

    monkeypatch.setattr(sys.stdin, "isatty", lambda: True)
    monkeypatch.setattr(cli, "_action_source", lambda: lambda: None)

    def fake_run_experiment(*_args, **_kwargs):
        error = KeyboardInterrupt()
        error.add_note("警告：退出清理未完全成功：失能失败：模拟")
        error.add_note(f"运行记录：{tmp_path / 'run'}")
        raise error

    monkeypatch.setattr(runtime, "run_experiment", fake_run_experiment)
    assert (
        cli.run(
            [
                "--execute",
                "--terminal.mode",
                "plain",
                "--output",
                str(tmp_path),
            ]
        )
        == 130
    )
    error_output = capsys.readouterr().err
    assert "退出清理已尝试完成" in error_output
    assert "失能失败：模拟" in error_output
    assert "运行记录：" in error_output


def test_help_lists_task_and_object_names(capsys: pytest.CaptureFixture[str]) -> None:
    """--help 可用并列出关键参数。"""
    from dmgripper_experiments import cli

    with pytest.raises(SystemExit) as excinfo:
        cli.run(["--help"])
    assert excinfo.value.code == 0
    assert "task-name" in capsys.readouterr().out


def test_legacy_entry_points_reject_with_migration_hint(capsys: pytest.CaptureFixture[str]) -> None:
    """旧入口明确拒绝执行并给出新命令。"""
    from dmgripper_experiments import legacy

    with pytest.raises(SystemExit) as cup_exit:
        legacy.cup_main()
    assert cup_exit.value.code == 2
    assert "dmgripper-run" in capsys.readouterr().err

    with pytest.raises(SystemExit) as demo_exit:
        legacy.force_demo_main()
    assert demo_exit.value.code == 2
    assert "dmgripper-run" in capsys.readouterr().err
