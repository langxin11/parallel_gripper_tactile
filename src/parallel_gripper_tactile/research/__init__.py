"""Hydra 科研入口所复用的组合配置、计划与执行能力。"""

from .configuration import (
    REPOSITORY_ROOT,
    ResearchConfigurationError,
    ResearchRunConfig,
    ResolvedResearchRun,
    resolve_research_run,
)
from .execution import ResearchRunOutcome, execute_research_run

__all__ = [
    "REPOSITORY_ROOT",
    "ResearchConfigurationError",
    "ResearchRunConfig",
    "ResearchRunOutcome",
    "ResolvedResearchRun",
    "execute_research_run",
    "resolve_research_run",
]
