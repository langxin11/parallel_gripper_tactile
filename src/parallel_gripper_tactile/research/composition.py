"""供脚本、CLI 与测试共享的 Hydra 单次运行组合入口。"""

from __future__ import annotations

from collections.abc import Sequence

from hydra import compose, initialize_config_dir
from hydra.core.global_hydra import GlobalHydra

from .configuration import REPOSITORY_ROOT, ResolvedResearchRun, resolve_research_run
from .hydra_support import register_resolvers, resolved_mapping


def compose_research_run(
    *,
    experiment: str,
    overrides: Sequence[str] = (),
) -> ResolvedResearchRun:
    """组合并校验一个命名实验，返回唯一的冻结领域配置。

    Args:
        experiment: ``configs/experiment`` 下的配置组选择，例如
            ``dm_gripper/force_tracking``。
        overrides: 应用在实验默认值之后的 Hydra 覆盖序列。

    Returns:
        runner、CLI 和产物快照共同消费的最终配置。
    """
    register_resolvers()
    config_directory = (REPOSITORY_ROOT / "configs").resolve()
    if GlobalHydra.instance().is_initialized():
        config = compose(
            config_name="run",
            overrides=[f"experiment={experiment}", *overrides],
        )
    else:
        with initialize_config_dir(version_base="1.3", config_dir=str(config_directory)):
            config = compose(
                config_name="run",
                overrides=[f"experiment={experiment}", *overrides],
            )
    return resolve_research_run(resolved_mapping(config))


__all__ = ["compose_research_run"]
