"""DMgripper 通用抓取实验命令行。

默认只验证配置并输出计划（dry-run），不导入运行时、不打开设备；
``--execute`` 是访问真机的显式开关。交互执行会按终端能力选择 Rich
或纯文本展示，非交互执行要求明确的自动启动与自动结束组合。
"""

from __future__ import annotations

import json
import queue
import sys
import threading
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, NoReturn, TextIO

from .config import ExperimentConfig, experiment_config_record, load_experiment_config
from .recording import create_run_directory


def _bootstrap(argv: Sequence[str]) -> tuple[Path | None, bool, bool, Path | None, list[str]]:
    """提取不属于 YAML 或 Tyro 配置的操作参数。"""
    config_path: Path | None = None
    execute = False
    bias = False
    output: Path | None = None
    remaining: list[str] = []
    index = 0
    while index < len(argv):
        argument = argv[index]
        if argument in {"--config", "--output"}:
            if index + 1 >= len(argv):
                raise ValueError(f"{argument} 需要路径")
            value = Path(argv[index + 1])
            if argument == "--config":
                config_path = value
            else:
                output = value
            index += 2
        elif argument == "--execute":
            execute = True
            index += 1
        elif argument == "--bias":
            bias = True
            index += 1
        else:
            remaining.append(argument)
            index += 1
    return config_path, execute, bias, output, remaining


@dataclass(frozen=True, slots=True)
class _InputFailure:
    """把后台输入异常安全地传回运行线程。"""

    error: Exception


_INPUT_CLOSED = object()
_ACTION_ALIASES = {"s": "start", "r": "release"}


def _normalize_action(text: str) -> str:
    """归一化命令并展开需要回车确认的单字符别名。"""
    action = text.strip().casefold()
    return _ACTION_ALIASES.get(action, action)


class _ActionSource:
    """从逐行输入或 Rich 固定输入区非阻塞交付命令。"""

    def __init__(self, stream: TextIO, on_edit: Callable[[str], None] | None) -> None:
        """启动输入线程；交互终端可切换为逐字符读取。"""
        self._stream = stream
        self._on_edit = on_edit
        self._actions: queue.SimpleQueue[object] = queue.SimpleQueue()
        self._restore: tuple[object, int, list[object]] | None = None
        self._restore_lock = threading.Lock()
        target = self._read_lines
        if on_edit is not None and bool(getattr(stream, "isatty", lambda: False)()):
            try:
                import termios
                import tty

                descriptor = stream.fileno()
                attributes = termios.tcgetattr(descriptor)
                tty.setcbreak(descriptor, termios.TCSANOW)
                self._restore = (termios, descriptor, attributes)
                target = self._read_characters
            except (AttributeError, OSError, termios.error):
                target = self._read_lines
        threading.Thread(target=target, name="dmgripper-input", daemon=True).start()

    def __call__(self) -> str | None:
        """返回当前已提交的一条命令。"""
        try:
            action = self._actions.get_nowait()
        except queue.Empty:
            return None
        if isinstance(action, _InputFailure):
            raise RuntimeError(f"交互输入读取失败：{action.error}") from action.error
        if action is _INPUT_CLOSED:
            raise RuntimeError("交互输入已关闭")
        assert isinstance(action, str)
        return action

    def close(self) -> None:
        """恢复逐字符模式之前的终端属性。"""
        with self._restore_lock:
            restore = self._restore
            self._restore = None
        if restore is not None:
            termios, descriptor, attributes = restore
            try:
                termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
            except (OSError, termios.error):
                pass
        if self._on_edit is not None:
            self._on_edit("")

    def _read_lines(self) -> None:
        """保留非终端流的原始逐行语义。"""
        try:
            for line in self._stream:
                action = _normalize_action(line)
                if action:
                    self._actions.put(action)
        except Exception as error:  # noqa: BLE001
            self._actions.put(_InputFailure(error))
        else:
            self._actions.put(_INPUT_CLOSED)

    def _read_characters(self) -> None:
        """在禁用内核回显的终端中维护 Rich 输入缓冲。"""
        buffer = ""
        try:
            while True:
                character = self._stream.read(1)
                if character == "":
                    self._actions.put(_INPUT_CLOSED)
                    break
                if character in {"\r", "\n"}:
                    action = _normalize_action(buffer)
                    buffer = ""
                    if self._on_edit is not None:
                        self._on_edit(buffer)
                    if action:
                        self._actions.put(action)
                elif character in {"\x7f", "\b"}:
                    buffer = buffer[:-1]
                    if self._on_edit is not None:
                        self._on_edit(buffer)
                elif character == "\x04":
                    self._actions.put(_INPUT_CLOSED)
                    break
                elif character.isprintable() and len(buffer) < 64:
                    buffer += character
                    if self._on_edit is not None:
                        self._on_edit(buffer)
        except Exception as error:  # noqa: BLE001
            self._actions.put(_InputFailure(error))
        finally:
            self.close()


def _action_source(
    stream: TextIO | None = None,
    *,
    on_edit: Callable[[str], None] | None = None,
) -> _ActionSource:
    """创建非阻塞操作事件来源。"""
    return _ActionSource(sys.stdin if stream is None else stream, on_edit)


def _emit(event: dict[str, object]) -> None:
    """输出一条 JSON 事件。"""
    print(json.dumps(event, ensure_ascii=False, allow_nan=False), flush=True)


def _print_exception_notes(error: BaseException) -> None:
    """把运行时附加的清理警告与记录目录醒目输出。"""
    for note in getattr(error, "__notes__", ()):
        print(note, file=sys.stderr)


def run(argv: Sequence[str] | None = None) -> int:
    """解析配置；默认只输出计划，显式 execute 才导入运行时。"""
    try:
        config_path, execute, bias, output, remaining = _bootstrap(
            list(sys.argv[1:] if argv is None else argv)
        )
        default = (
            load_experiment_config(config_path) if config_path is not None else ExperimentConfig()
        )
        import tyro

        config = tyro.cli(
            ExperimentConfig,
            args=remaining,
            default=default,
            description=(
                "DMgripper 通用抓取实验。默认只检查配置；--config PATH 加载严格 YAML，"
                "--output PATH 指定输出根目录，--bias 请求空载清零，--execute 在终端执行。"
                "运行时输入 start、status 或 release，并按 Enter 提交。"
            ),
        )
        record = experiment_config_record(config)
        if not execute:
            _emit(
                {
                    "mode": "dry-run",
                    "dm_port": config.hardware.dm_port,
                    "tactile_port": config.hardware.tactile_port,
                    "clear_bias": bias,
                    "config": record,
                }
            )
            return 0

        interactive = sys.stdin.isatty()
        if not interactive:
            if not config.lifecycle.auto_start or config.lifecycle.on_finished != "return":
                raise ValueError(
                    "非交互 --execute 要求 lifecycle.auto_start=true 且 on_finished=return；"
                    "交互实验请在终端运行"
                )

        output_root = output if output is not None else Path(config.output.root)
        output_directory = create_run_directory(output_root, config)

        from .runtime import run_experiment
        from .terminal import TerminalDisplay

        terminal_mode = config.terminal.mode
        if terminal_mode == "auto" and not interactive and not sys.stdout.isatty():
            terminal_mode = "plain"
        terminal = TerminalDisplay(terminal_mode, refresh_hz=config.terminal.refresh_hz)

        def event_sink(event: dict[str, object]) -> None:
            """Rich 正常时由面板显示；降级后立即恢复 JSON 事件输出。"""
            if not terminal.uses_rich:
                _emit(event)

        if interactive and terminal.uses_rich:
            action_source = _action_source(on_edit=terminal.set_command_input)
        elif interactive:
            action_source = _action_source()
        else:

            def action_source() -> None:
                """非交互执行不产生人工命令。"""
                return None

        try:
            result = run_experiment(
                config,
                output_directory=output_directory,
                clear_bias=bias,
                action_source=action_source,
                event_sink=event_sink,
                terminal=terminal,
                input_config_path=config_path,
            )
        finally:
            close_action_source = getattr(action_source, "close", None)
            if close_action_source is not None:
                close_action_source()
        if not terminal.uses_rich:
            _emit({"event": "complete", **result})
        else:
            disable_confirmed = result["disable_confirmed"]
            if disable_confirmed is True:
                disable_label = "已确认"
            elif disable_confirmed is False:
                disable_label = "未确认"
            else:
                disable_label = "未曾使能"
            print(
                "实验结束："
                f"状态={result['status']}；电机失能={disable_label}；"
                f"运行目录={result['output_directory']}",
                flush=True,
            )
        return 0
    except KeyboardInterrupt as error:
        print("实验已中断；若设备运行已经开始，退出清理已尝试完成。", file=sys.stderr)
        _print_exception_notes(error)
        return 130
    except Exception as error:  # noqa: BLE001
        print(f"实验失败：{error}", file=sys.stderr)
        _print_exception_notes(error)
        return 1


def main(argv: Sequence[str] | None = None) -> NoReturn:
    """运行通用抓取实验命令行。"""
    raise SystemExit(run(argv))
