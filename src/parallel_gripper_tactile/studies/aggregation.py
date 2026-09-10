"""研究脚本共享的声明式行聚合工具。

各研究脚本（``scripts/experiments``）把逐次运行的 ``list[dict]`` 行按分组键汇总成
聚合行。历史上每个脚本手写一套循环，语义在 None／NaN／inf 处理上有细微差别；
本模块用 polars 表达式把这些口径固化为可声明的列规格（column spec），由
:func:`aggregate_records` 统一解释执行。

语义契约（与被替换的手写实现逐点对齐）：

- 分组键在构建 DataFrame 时统一字符串化，输出行序等于各组首次出现顺序；
- ``count``／``bool_sum`` 等计数列为整数，``first`` 透传组内首行原值；
- ``finite_mean``／``finite_std``：先按 ``is_finite`` 过滤（None、NaN、inf 一并剔除），
  无有限值时均值为 ``None``，有限值不足 2 个时标准差为 ``None``；
- ``optional_mean``／``optional_std``：仅过滤 ``None``，不滤 NaN／inf；
  NaN 会像 ``statistics.fmean`` 一样传播为 NaN；
- ``plain_mean``：不做任何过滤，NaN 传播语义同上；
- ``nan_mean``／``nan_std``：NaN 感知统计（对应阶跃瞬态指标的 numpy
  ``nanmean``／``nanstd(ddof=1)`` 口径），None 与 NaN 都被忽略而 inf 保留，
  组内无有效值或标准差自由度不足时返回 NaN 而非 ``None``；
- ``bool_rate`` 输出检测率，可用 ``zero_when`` 在指定首行布尔列为真时强制 0.0；
- ``threshold_count`` 统计大于等于阈值的运行数（NaN 视为不满足）。

浮点说明：均值与 ``statistics.fmean``、标准差与 ``statistics.stdev`` 在个别输入上
可能出现 ULP 级差异；调用方按容差比较时应使用相对 1e-12 量级的阈值。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TypeAlias

import polars as pl

_NAN = math.nan


@dataclass(frozen=True, slots=True)
class _KeyColumn:
    """输出分组键本身（已字符串化）的列。"""

    name: str


@dataclass(frozen=True, slots=True)
class _Count:
    """输出组内行数的列。"""

    alias: str = "runs"

    @property
    def output_name(self) -> str:
        """返回输出列名。"""
        return self.alias

    def _sources(self) -> tuple[str, ...]:
        """返回引用的来源列名。"""
        return ()

    def _agg_expr(self) -> pl.Expr:
        """返回组内行数的聚合表达式。"""
        return pl.len()


@dataclass(frozen=True, slots=True)
class _FirstValue:
    """透传组内首行原值的列。"""

    source: str
    alias: str | None = None
    as_bool: bool = False

    @property
    def output_name(self) -> str:
        """返回输出列名，未指定别名时与来源列同名。"""
        return self.source if self.alias is None else self.alias

    def _sources(self) -> tuple[str, ...]:
        """返回引用的来源列名。"""
        return (self.source,)

    def _agg_expr(self) -> pl.Expr:
        """返回取首行的聚合表达式。"""
        column = pl.col(self.source)
        if self.as_bool:
            return column.cast(pl.Boolean).first()
        return column.first()


@dataclass(frozen=True, slots=True)
class _BoolSum:
    """统计组内真值行数的列。"""

    source: str
    alias: str

    @property
    def output_name(self) -> str:
        """返回输出列名。"""
        return self.alias

    def _sources(self) -> tuple[str, ...]:
        """返回引用的来源列名。"""
        return (self.source,)

    def _agg_expr(self) -> pl.Expr:
        """返回布尔求和的聚合表达式。"""
        return pl.col(self.source).cast(pl.Boolean).sum()


@dataclass(frozen=True, slots=True)
class _BoolRate:
    """输出真值占比的列，可按另一首行布尔列强制为 0.0。"""

    source: str
    alias: str
    zero_when: tuple[str, bool] | None = None

    @property
    def output_name(self) -> str:
        """返回输出列名。"""
        return self.alias

    def _sources(self) -> tuple[str, ...]:
        """返回引用的来源列名。"""
        if self.zero_when is None:
            return (self.source,)
        return (self.source, self.zero_when[0])

    def _agg_expr(self) -> pl.Expr:
        """返回真值占比（或条件 0.0）的聚合表达式。"""
        rate = pl.col(self.source).cast(pl.Boolean).sum() / pl.len()
        if self.zero_when is None:
            return rate
        source, expected = self.zero_when
        gate = pl.col(source).cast(pl.Boolean).first() == expected
        return pl.when(gate).then(pl.lit(0.0)).otherwise(rate)


@dataclass(frozen=True, slots=True)
class _ThresholdCount:
    """统计数值列大于等于阈值的行数的列。"""

    source: str
    alias: str
    at_least: float

    @property
    def output_name(self) -> str:
        """返回输出列名。"""
        return self.alias

    def _sources(self) -> tuple[str, ...]:
        """返回引用的来源列名。"""
        return (self.source,)

    def _agg_expr(self) -> pl.Expr:
        """返回阈值计数的聚合表达式。

        polars 把 NaN 当作最大值参与比较，必须显式排除 NaN 才能匹配
        Python ``float`` 比较语义；``None`` 行同样不计数。
        """
        column = pl.col(self.source)
        return ((column >= self.at_least) & column.is_not_nan()).sum()


@dataclass(frozen=True, slots=True)
class _Minimum:
    """输出数值列最小值的列。"""

    source: str
    alias: str

    @property
    def output_name(self) -> str:
        """返回输出列名。"""
        return self.alias

    def _sources(self) -> tuple[str, ...]:
        """返回引用的来源列名。"""
        return (self.source,)

    def _agg_expr(self) -> pl.Expr:
        """返回最小值的聚合表达式。

        polars 的 ``min`` 跳过 NaN 与 ``None``；与 Python ``min`` 在含 NaN 输入上
        行为不同，本规格只面向实测中均为有限值的列。
        """
        return pl.col(self.source).min()


@dataclass(frozen=True, slots=True)
class _Stat:
    """按指定过滤口径输出均值或样本标准差（ddof=1）的列。"""

    source: str
    alias: str | None = None
    statistic: str = "mean"
    filtering: str = "plain"
    empty_as_nan: bool = False

    @property
    def output_name(self) -> str:
        """返回输出列名，未指定别名时为 ``<来源>_<统计量>``。"""
        suffix = self.statistic
        return f"{self.source}_{suffix}" if self.alias is None else self.alias

    def _sources(self) -> tuple[str, ...]:
        """返回引用的来源列名。"""
        return (self.source,)

    def _agg_expr(self) -> pl.Expr:
        """返回过滤后聚合的统计表达式。

        Raises:
            ValueError: 统计量或过滤口径取了未定义值。
        """
        column = pl.col(self.source)
        if self.filtering == "finite":
            column = column.filter(column.is_finite())
        elif self.filtering == "non_null":
            column = column.filter(column.is_not_null())
        elif self.filtering == "non_nan":
            column = column.filter(column.is_not_nan())
        elif self.filtering != "plain":
            raise ValueError(f"未知过滤口径：{self.filtering}")
        if self.statistic == "mean":
            value = column.mean()
        elif self.statistic == "std":
            value = column.std(ddof=1)
        else:
            raise ValueError(f"未知统计量：{self.statistic}")
        if self.empty_as_nan:
            # 空组或自由度不足时 polars 返回 null，NaN 感知口径要求输出 NaN。
            return value.fill_null(_NAN)
        return value


ColumnSpec: TypeAlias = (
    _KeyColumn | _Count | _FirstValue | _BoolSum | _BoolRate | _ThresholdCount | _Minimum | _Stat
)


def key(name: str) -> _KeyColumn:
    """声明输出分组键本身的列。

    Args:
        name: 分组键列名，同时也是输出列名。

    Returns:
        引用分组键（字符串化后）的列规格。
    """
    return _KeyColumn(name)


def count(alias: str = "runs") -> _Count:
    """声明输出组内行数的列。

    Args:
        alias: 输出列名。

    Returns:
        输出 ``pl.len()`` 的列规格。
    """
    return _Count(alias)


def first(source: str, alias: str | None = None) -> _FirstValue:
    """声明透传组内首行原值的列。

    Args:
        source: 来源列名。
        alias: 输出列名，缺省与来源列同名。

    Returns:
        取首行原值的列规格。
    """
    return _FirstValue(source, alias)


def first_bool(source: str, alias: str | None = None) -> _FirstValue:
    """声明透传组内首行值并转为布尔型的列。

    Args:
        source: 来源列名。
        alias: 输出列名，缺省与来源列同名。

    Returns:
        取首行并 ``cast(Boolean)`` 的列规格。
    """
    return _FirstValue(source, alias, as_bool=True)


def bool_sum(source: str, alias: str) -> _BoolSum:
    """声明统计组内真值行数的列。

    Args:
        source: 来源布尔列名。
        alias: 输出列名。

    Returns:
        布尔求和的列规格，输出为整数。
    """
    return _BoolSum(source, alias)


def bool_rate(source: str, alias: str, *, zero_when: tuple[str, bool] | None = None) -> _BoolRate:
    """声明输出真值占比的列。

    Args:
        source: 来源布尔列名。
        alias: 输出列名。
        zero_when: 可选的 ``(列名, 期望值)``；当该列首行布尔值等于期望值时
            输出固定 0.0（例如期望起滑的正例场景误报率恒为 0）。

    Returns:
        真值占比（或条件 0.0）的列规格。
    """
    return _BoolRate(source, alias, zero_when)


def threshold_count(source: str, alias: str, *, at_least: float) -> _ThresholdCount:
    """声明统计数值列大于等于阈值的行数的列。

    Args:
        source: 来源数值列名。
        alias: 输出列名。
        at_least: 阈值，比较为闭区间（大于等于计入）。

    Returns:
        阈值计数的列规格；NaN 与 ``None`` 均不满足阈值。
    """
    return _ThresholdCount(source, alias, at_least)


def minimum(source: str, alias: str) -> _Minimum:
    """声明输出数值列最小值的列。

    Args:
        source: 来源数值列名，要求组内为有限值。
        alias: 输出列名。

    Returns:
        最小值的列规格。
    """
    return _Minimum(source, alias)


def plain_mean(source: str, alias: str | None = None) -> _Stat:
    """声明无过滤均值的列（NaN 像 ``statistics.fmean`` 一样传播）。

    Args:
        source: 来源数值列名，来源行不应为 ``None``。
        alias: 输出列名，缺省为 ``<来源>_mean``。

    Returns:
        无过滤均值的列规格。
    """
    return _Stat(source, alias, statistic="mean", filtering="plain")


def finite_mean(source: str, alias: str | None = None) -> _Stat:
    """声明有限值过滤均值的列（None、NaN、inf 均剔除；无有限值为 ``None``）。

    Args:
        source: 来源数值列名。
        alias: 输出列名，缺省为 ``<来源>_mean``。

    Returns:
        有限值均值的列规格。
    """
    return _Stat(source, alias, statistic="mean", filtering="finite")


def finite_std(source: str, alias: str | None = None) -> _Stat:
    """声明有限值过滤样本标准差（ddof=1）的列（有限值不足 2 个为 ``None``）。

    Args:
        source: 来源数值列名。
        alias: 输出列名，缺省为 ``<来源>_std``。

    Returns:
        有限值样本标准差的列规格。
    """
    return _Stat(source, alias, statistic="std", filtering="finite")


def optional_mean(source: str, alias: str | None = None) -> _Stat:
    """声明仅过滤 ``None`` 的均值列（不滤 NaN／inf，NaN 传播为 NaN）。

    Args:
        source: 来源数值列名。
        alias: 输出列名，缺省为 ``<来源>_mean``。

    Returns:
        ``None`` 过滤均值的列规格；组内全为 ``None`` 时输出 ``None``。
    """
    return _Stat(source, alias, statistic="mean", filtering="non_null")


def optional_std(source: str, alias: str | None = None) -> _Stat:
    """声明仅过滤 ``None`` 的样本标准差（ddof=1）列（NaN 传播为 NaN）。

    Args:
        source: 来源数值列名。
        alias: 输出列名，缺省为 ``<来源>_std``。

    Returns:
        ``None`` 过滤样本标准差的列规格；有效值不足 2 个时输出 ``None``。
    """
    return _Stat(source, alias, statistic="std", filtering="non_null")


def nan_mean(source: str, alias: str | None = None) -> _Stat:
    """声明 NaN 感知均值的列（忽略 ``None`` 与 NaN、保留 inf，无有效值为 NaN）。

    Args:
        source: 来源数值列名。
        alias: 输出列名，缺省为 ``<来源>_mean``。

    Returns:
        NaN 感知均值的列规格，输出恒为浮点数。
    """
    return _Stat(source, alias, statistic="mean", filtering="non_nan", empty_as_nan=True)


def nan_std(source: str, alias: str | None = None) -> _Stat:
    """声明 NaN 感知样本标准差（ddof=1）的列（对应 numpy ``nanstd`` 口径）。

    有效值不足 2 个时输出 NaN；inf 参与统计时方差未定义、同样传播为 NaN。

    Args:
        source: 来源数值列名。
        alias: 输出列名，缺省为 ``<来源>_std``。

    Returns:
        NaN 感知样本标准差的列规格，输出恒为浮点数。
    """
    return _Stat(source, alias, statistic="std", filtering="non_nan", empty_as_nan=True)


def aggregate_records(
    rows: list[dict[str, object]],
    *,
    keys: tuple[str, ...],
    columns: tuple[ColumnSpec, ...],
) -> list[dict[str, object]]:
    """按声明式列规格把逐次运行行聚合为汇总行。

    Args:
        rows: 逐次运行的字典行列表。
        keys: 1～3 个分组键列名；键值在分组时统一按 ``str()`` 语义字符串化，
            输出行序为各分组首次出现的顺序。
        columns: 按输出顺序排列的列规格，键列可穿插其中。
            同一来源列可重复声明（例如同时输出均值与标准差）。

    Returns:
        聚合行列表；行序为各分组首次出现的顺序，列序与 ``columns``
        声明顺序一致。``rows`` 为空时返回空列表。
    """
    if not rows:
        return []
    if not 1 <= len(keys) <= 3:
        raise ValueError("分组键数量必须在 1 到 3 之间。")
    frame = pl.DataFrame(rows)
    # 输入行整体缺失的指标列按全 None 处理，与部分行缺键时 polars 的
    # null 填充语义一致（旧手写实现用 ``row.get(...) is None`` 得到同样的 None）。
    referenced = {
        column for spec in columns if not isinstance(spec, _KeyColumn) for column in spec._sources()
    }
    missing = [column for column in sorted(referenced) if column not in frame.columns]
    if missing:
        frame = frame.with_columns(
            pl.lit(None, dtype=pl.Float64).alias(column) for column in missing
        )
    key_aliases = {name: f"__study_key_{index}" for index, name in enumerate(keys)}
    # 分组键在 group_by 表达式中字符串化；原始列保持原 dtype，
    # 以便 ``first`` 等透传规格仍取到未字符串化的首行原值。
    grouped = frame.group_by(
        [pl.col(name).cast(pl.String).alias(alias) for name, alias in key_aliases.items()],
        maintain_order=True,
    )
    agg_exprs = [
        spec._agg_expr().alias(spec.output_name)
        for spec in columns
        if not isinstance(spec, _KeyColumn)
    ]
    selects: list[pl.Expr] = []
    for spec in columns:
        if isinstance(spec, _KeyColumn):
            selects.append(pl.col(key_aliases[spec.name]).alias(spec.name))
        else:
            selects.append(pl.col(spec.output_name))
    return grouped.agg(agg_exprs).select(selects).to_dicts()


__all__ = [
    "aggregate_records",
    "bool_rate",
    "bool_sum",
    "count",
    "finite_mean",
    "finite_std",
    "first",
    "first_bool",
    "key",
    "minimum",
    "nan_mean",
    "nan_std",
    "optional_mean",
    "optional_std",
    "plain_mean",
    "threshold_count",
]
