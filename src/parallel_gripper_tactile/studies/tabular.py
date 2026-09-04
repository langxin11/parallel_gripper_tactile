"""提供 study 结果表的 CSV 与 Parquet 兼容读写。"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import pyarrow as pa
import pyarrow.parquet as pq
from pydantic import BaseModel


def write_rows_csv(csv_path: Path, rows: list[dict[str, object]]) -> Path:
    """写入 study 结果的 CSV 表。

    Args:
        csv_path: CSV 输出路径。
        rows: 字段同构的结果行。

    Returns:
        已生成的 CSV 路径。

    Raises:
        ValueError: 结果行为空时抛出。
    """
    if not rows:
        raise ValueError("不能写入空的 study 结果行。")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


def write_rows_csv_and_parquet(csv_path: Path, rows: list[dict[str, object]]) -> tuple[Path, Path]:
    """同时写入 CSV 和使用 Zstd 压缩的同名 Parquet 表。

    Args:
        csv_path: CSV 输出路径；Parquet 使用相同文件名和 `.parquet` 后缀。
        rows: 字段同构的结果行；其中的 ``None`` 和 ``NaN`` 保持为 Parquet 可表达的缺失值。

    Returns:
        已生成的 CSV 与 Parquet 路径。

    Raises:
        ValueError: 结果行为空时抛出。
    """
    write_rows_csv(csv_path, rows)

    parquet_path = csv_path.with_suffix(".parquet")
    pq.write_table(pa.Table.from_pylist(rows), parquet_path, compression="zstd")
    return csv_path, parquet_path


def write_resolved_config(path: Path, config: BaseModel) -> Path:
    """写入包含解析路径和默认值的 study 配置快照。

    Args:
        path: `study.resolved.json` 输出路径。
        config: 已完成路径解析的 study Pydantic 配置。

    Returns:
        已生成的配置快照路径。
    """
    path.write_text(
        json.dumps(config.model_dump(mode="json"), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return path


def read_trace_rows(run_directory: Path) -> list[dict[str, Any]]:
    """读取单次运行的 trace，优先使用 Parquet 并兼容历史 CSV。

    Args:
        run_directory: 包含 ``trace.parquet`` 或 ``trace.csv`` 的单次运行目录。

    Returns:
        trace 的行记录。

    Raises:
        FileNotFoundError: 两种 trace 文件均不存在时抛出。
    """
    parquet_path = run_directory / "trace.parquet"
    if parquet_path.is_file():
        return pq.read_table(parquet_path).to_pylist()

    csv_path = run_directory / "trace.csv"
    if csv_path.is_file():
        with csv_path.open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))
    raise FileNotFoundError(f"找不到运行 trace：{parquet_path} 或 {csv_path}")
