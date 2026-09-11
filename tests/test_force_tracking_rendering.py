"""验证历史力跟踪重绘的输入保护与产物账本。"""

import csv
import json
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq
import pytest

from parallel_gripper_tactile.runners.rendering import render_force_tracking_run


@pytest.mark.parametrize("trace_format", ["parquet", "csv"])
def test_replot_preserves_sources_and_uses_original_metrics(
    tmp_path: Path, trace_format: str, fast_plot_render: None
) -> None:
    """旧数据缺诊断列仍可重绘，原 trace、指标和 run 账本保持逐字节不变。"""
    source = tmp_path / "run"
    source.mkdir()
    rows = [
        {
            "time_s": t,
            "phase": "track_reference",
            "tracking_time_s": t,
            "target_normal_force_n": 2.0,
            "filtered_normal_force_n": 1.0,
        }
        for t in (0.0, 1.0)
    ]
    trace = source / f"trace.{trace_format}"
    if trace_format == "parquet":
        pq.write_table(pa.Table.from_pylist(rows), trace)
    else:
        with trace.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    # 数值刻意不同于稀疏 trace 的误差，防止绘图端重算指标。
    (source / "metrics.json").write_text('{"rmse_n": 0.123}', encoding="utf-8")
    (source / "manifest.json").write_text('{"artifacts": []}', encoding="utf-8")
    before = {path: path.read_bytes() for path in source.iterdir() if path.is_file()}

    output = render_force_tracking_run(source, tactile_detail=True)

    assert output.parent == source / "plots" / "replots"
    assert {path: path.read_bytes() for path in before} == before
    manifest = json.loads((output / "rendering_manifest.json").read_text(encoding="utf-8"))
    assert manifest["artifacts"] == ["tracking.png"]
    assert set(manifest["source_sha256"]) == {trace.name, "metrics.json"}
    assert any("刚度有效性" in note for note in manifest["notes"])
    assert any("触觉细节不可用" in note for note in manifest["notes"])
    assert not list(output.glob("*.pdf"))
    with pytest.raises(FileExistsError):
        render_force_tracking_run(source, output_directory=output)


def test_replot_requires_original_metrics(tmp_path: Path) -> None:
    """缺少原指标时拒绝用降采样轨迹替代统计。"""
    pq.write_table(pa.table({"time_s": [0.0]}), tmp_path / "trace.parquet")
    with pytest.raises(FileNotFoundError):
        render_force_tracking_run(tmp_path)
    assert not (tmp_path / "plots").exists()
