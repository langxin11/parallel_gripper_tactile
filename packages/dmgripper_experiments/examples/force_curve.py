"""给定时间力曲线任务的示例：构造配置并打印 dry-run 计划。

示例调用与 CLI 相同的公共接口，默认离线：只验证配置、不创建设备、
不使能电机、不生成正式运行产物。
"""

from __future__ import annotations

import json

from dmgripper_experiments.config import (
    CurveReferenceConfig,
    ExperimentConfig,
    ReferenceConfig,
    WaypointConfig,
    experiment_config_record,
)


def build_curve_task() -> ExperimentConfig:
    """构造带上升段的力曲线任务配置。"""
    return ExperimentConfig(
        reference=ReferenceConfig(
            curve=CurveReferenceConfig(
                interpolation="smoothstep",
                waypoints=(
                    WaypointConfig(t_s=0.0, force_n=0.5),
                    WaypointConfig(t_s=4.0, force_n=0.8),
                    WaypointConfig(t_s=8.0, force_n=0.8),
                ),
            )
        )
    )


def main() -> None:
    """验证配置并输出计划 JSON。"""
    config = build_curve_task()
    print(
        json.dumps(
            {"mode": "dry-run", "config": experiment_config_record(config)},
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
