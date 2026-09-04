"""验证 study 结果表的双格式持久化与 trace 兼容读取。"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import pyarrow.parquet as pq
from pydantic import BaseModel

from parallel_gripper_tactile.studies.tabular import (
    read_trace_rows,
    write_resolved_config,
    write_rows_csv_and_parquet,
)


class _Config(BaseModel):
    """构造带默认值与路径的最小 study 配置。"""

    profile: Path
    repeats: int = 3


def test_write_rows_csv_and_parquet_preserves_nullable_values_with_zstd(tmp_path: Path) -> None:
    """CSV 继续输出，Parquet 使用 Zstd 并保留 None 与 NaN。"""
    csv_path, parquet_path = write_rows_csv_and_parquet(
        tmp_path / "summary.csv",
        [
            {"name": "first", "optional": None, "measurement": math.nan},
            {"name": "second", "optional": 2.0, "measurement": 1.5},
        ],
    )

    assert csv_path.is_file()
    assert parquet_path.is_file()
    assert pq.ParquetFile(parquet_path).metadata.row_group(0).column(0).compression == "ZSTD"
    rows = pq.read_table(parquet_path).to_pylist()
    assert rows[0]["optional"] is None
    assert math.isnan(rows[0]["measurement"])


def test_read_trace_rows_prefers_parquet_and_falls_back_to_csv(tmp_path: Path) -> None:
    """新 trace 优先使用 Parquet，历史运行仍可读取 CSV。"""
    run_directory = tmp_path / "run"
    run_directory.mkdir()
    with (run_directory / "trace.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("phase", "value"))
        writer.writeheader()
        writer.writerow({"phase": "track_reference", "value": "csv"})

    assert read_trace_rows(run_directory) == [{"phase": "track_reference", "value": "csv"}]

    write_rows_csv_and_parquet(
        run_directory / "trace.csv", [{"phase": "track_reference", "value": "parquet"}]
    )

    assert read_trace_rows(run_directory) == [{"phase": "track_reference", "value": "parquet"}]


def test_write_resolved_config_expands_defaults_and_serializes_paths(tmp_path: Path) -> None:
    """解析后配置快照写入完整默认值和 JSON 路径。"""
    config_path = write_resolved_config(
        tmp_path / "study.resolved.json", _Config(profile=(tmp_path / "profile.yaml").resolve())
    )

    assert config_path.read_text(encoding="utf-8") == (
        '{\n  "profile": "' + str((tmp_path / "profile.yaml").resolve()) + '",\n  "repeats": 3\n}\n'
    )
