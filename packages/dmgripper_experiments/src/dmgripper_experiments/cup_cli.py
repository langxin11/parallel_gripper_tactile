"""DMgripper 倒水实验命令行。"""

from __future__ import annotations

import json
import queue
import sys
import threading
from collections.abc import Callable, Sequence
from datetime import datetime
from pathlib import Path
from typing import NoReturn

import tyro

from .cup_config import CupConfig, cup_config_record, load_cup_config


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


def _action_source() -> Callable[[], str | None]:
    """创建非阻塞的交互命令来源。"""
    actions: queue.SimpleQueue[str] = queue.SimpleQueue()

    def read_stdin() -> None:
        """持续读取完整命令行。"""
        for line in sys.stdin:
            actions.put(line.strip())
        actions.put("__input_closed__")

    threading.Thread(target=read_stdin, daemon=True).start()

    def next_action() -> str | None:
        """返回当前已有的一条命令。"""
        try:
            return actions.get_nowait()
        except queue.Empty:
            return None

    return next_action


def _emit(event: dict[str, object]) -> None:
    """输出一条 JSON 事件。"""
    print(json.dumps(event, ensure_ascii=False, allow_nan=False), flush=True)


def run(argv: Sequence[str] | None = None) -> int:
    """解析配置；默认只输出计划，显式 execute 才导入运行时。"""
    try:
        config_path, execute, bias, output, remaining = _bootstrap(
            list(sys.argv[1:] if argv is None else argv)
        )
        default = load_cup_config(config_path) if config_path is not None else CupConfig()
        config = tyro.cli(
            CupConfig,
            args=remaining,
            default=default,
            description=(
                "DMgripper 倒水实验。默认只检查配置；--config PATH 加载 YAML，"
                "--output PATH 指定独占结果目录，--bias 请求空载清零，"
                "--execute 在交互终端执行。运行时输入 ready、status 或 release。"
            ),
        )
        output_directory = output or Path(
            f"outputs/real/dmgripper_cup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        if not execute:
            _emit(
                {
                    "mode": "dry-run",
                    "dm_port": config.dm_port,
                    "tactile_port": config.tactile_port,
                    "clear_bias": bias,
                    "output_directory": str(output_directory),
                    "config": cup_config_record(config),
                }
            )
            return 0

        if not sys.stdin.isatty():
            raise ValueError("--execute 需要交互终端，以便确认撤手与释放")

        from .cup_runtime import run_cup

        result = run_cup(
            config,
            output_directory=output_directory,
            clear_bias=bias,
            action_source=_action_source(),
            event_sink=_emit,
        )
        _emit({"event": "complete", **result})
        return 0
    except KeyboardInterrupt:
        print("倒水实验已中断，运行时正在执行退出清理。", file=sys.stderr)
        return 130
    except Exception as error:  # noqa: BLE001
        print(f"倒水实验失败：{error}", file=sys.stderr)
        return 1


def main(argv: Sequence[str] | None = None) -> NoReturn:
    """运行倒水实验命令行。"""
    raise SystemExit(run(argv))
