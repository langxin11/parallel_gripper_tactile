"""触觉动态增力任务的示例：构造配置并打印 dry-run 计划。

示例调用与 CLI 相同的公共接口，默认离线：只验证配置、不创建设备、
不使能电机、不生成正式运行产物。
"""

from __future__ import annotations

import json

from dmgripper_experiments.config import (
    AdaptiveReferenceConfig,
    ControllerConfig,
    ExperimentConfig,
    ReferenceConfig,
    experiment_config_record,
)


def build_adaptive_task() -> ExperimentConfig:
    """构造 PID 控制器与动态增力目标的任务配置。"""
    return ExperimentConfig(
        metadata=ExperimentConfig().metadata,
        controller=ControllerConfig(kind="pid"),
        reference=ReferenceConfig(
            adaptive=AdaptiveReferenceConfig(
                initial_force_n=0.5,
                duration_s=8.0,
            )
        ),
    )


def main() -> None:
    """验证配置并输出计划 JSON。"""
    config = build_adaptive_task()
    print(
        json.dumps(
            {"mode": "dry-run", "config": experiment_config_record(config)},
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
