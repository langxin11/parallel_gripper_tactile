"""按设备时间回放逐触点记录，仅作旁路诊断，不创建硬件会话。"""

import argparse
import csv
from dataclasses import replace
import json
from pathlib import Path

from dm_grasp_core.grasp.unified import UnifiedAdaptiveConfig, UnifiedAdaptivePolicy

from .config import load_experiment_config


def replay_tactile(source: Path, destination: Path, config: UnifiedAdaptiveConfig) -> int:
    """把原始 tactile.jsonl 因果回放为诊断 CSV，拒绝覆盖已有文件。

    回放始终撤销风险增力和摩擦更新权限，也不模拟执行器。输入应为已
    核对坐标与符号的一段连续抓取记录；新抓取须单独回放。返回消费的新样本数。
    """
    policy = UnifiedAdaptivePolicy(
        replace(config, risk_enabled=False, friction_update_enabled=False)
    )
    count = 0
    previous_us = None
    with (
        source.open(encoding="utf-8") as incoming,
        destination.open("x", encoding="utf-8", newline="") as outgoing,
    ):
        writer = None
        for line in incoming:
            sample = json.loads(line)
            timestamp = sample["timestamp_us"]
            if isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp < 0:
                raise ValueError("timestamp_us 必须为非负整数")
            if previous_us == timestamp:
                continue
            command = policy.update(
                sample["left_taxel_forces_n"],
                sample["right_taxel_forces_n"],
                time_s=timestamp * 1e-6,
                measured_force_n=(sample["left_force_n"] + sample["right_force_n"]) / 2,
                enabled=False,
            )
            row = {
                "timestamp_us": timestamp,
                "diagnostic_only": True,
                "raw_load_requirement_n": command.load.raw_target_force_n,
                **command.trace_fields(),
            }
            if writer is None:
                writer = csv.DictWriter(outgoing, fieldnames=list(row))
                writer.writeheader()
            writer.writerow(row)
            previous_us = timestamp
            count += 1
    if not count:
        raise ValueError("输入记录没有可回放样本")
    return count


def main() -> None:
    """提供只读输入、独占输出的离线入口。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="连续抓取段的 tactile.jsonl")
    parser.add_argument("destination", type=Path, help="尚不存在的诊断 CSV")
    parser.add_argument(
        "--config", type=Path, required=True, help="包含 adaptive.unified 的硬件配置"
    )
    args = parser.parse_args()
    experiment = load_experiment_config(args.config)
    if not experiment.unified_adaptive_enabled:
        parser.error("配置必须启用 reference.adaptive.unified")
    count = replay_tactile(args.source, args.destination, experiment.reference.adaptive.unified)
    print(f"已旁路回放 {count} 个新样本；未运行硬件或开放控制权限。")


if __name__ == "__main__":
    main()
