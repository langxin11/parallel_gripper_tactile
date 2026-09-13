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
from typing import NoReturn, TextIO

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


def _action_source(stream: TextIO | None = None):
    """创建非阻塞的交互命令来源；输入线程不访问串口。

    完整输入行会去除首尾空白并按 Unicode 规则折叠大小写；空行忽略。
    后台读取异常不会静默丢失，而是在运行线程下次轮询时明确抛出。
    """
    input_stream = sys.stdin if stream is None else stream
    actions: queue.SimpleQueue[object] = queue.SimpleQueue()

    def read_stdin() -> None:
        """持续读取完整命令行。"""
        try:
            for line in input_stream:
                action = line.strip().casefold()
                if action:
                    actions.put(action)
        except Exception as error:  # noqa: BLE001
            actions.put(_InputFailure(error))
        else:
            actions.put(_INPUT_CLOSED)

    threading.Thread(target=read_stdin, name="dmgripper-input", daemon=True).start()

    def next_action() -> str | None:
        """返回当前已有的一条命令。"""
        try:
            action = actions.get_nowait()
        except queue.Empty:
            return None
        if isinstance(action, _InputFailure):
            raise RuntimeError(f"交互输入读取失败：{action.error}") from action.error
        if action is _INPUT_CLOSED:
            raise RuntimeError("交互输入已关闭")
        assert isinstance(action, str)
        return action

    return next_action


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

        action_source = _action_source() if interactive else lambda: None
        result = run_experiment(
            config,
            output_directory=output_directory,
            clear_bias=bias,
            action_source=action_source,
            event_sink=event_sink,
            terminal=terminal,
            input_config_path=config_path,
        )
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
