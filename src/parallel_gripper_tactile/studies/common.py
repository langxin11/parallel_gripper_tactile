"""各力跟踪研究 protocol 共用的配置基类与种子扫描模型。"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field


class StudyConfigError(ValueError):
    """当 study YAML 无法解析或未通过 schema 校验时抛出。"""


class _StudyModel(BaseModel):
    """拒绝未知字段的不可变 study 配置基类。"""

    model_config = ConfigDict(extra="forbid", frozen=True)


class SeedSweep(_StudyModel):
    """连续的、可复现的传感器噪声种子范围。"""

    start: int = Field(default=0, ge=0)
    count: int = Field(default=3, ge=1)

    def values(self) -> tuple[int, ...]:
        """返回此次 study 要执行的种子。"""
        return tuple(range(self.start, self.start + self.count))
