"""从既有力跟踪产物独立重绘，不加载模型或重新统计指标。"""

from __future__ import annotations

import csv
from datetime import UTC, datetime
from hashlib import sha256
import json
from pathlib import Path
from uuid import uuid4

import yaml

from ..visualization.force_tracking import render_run_artifacts


def render_force_tracking_run(
    run_directory: Path,
    *,
    output_directory: Path | None = None,
    tactile_detail: bool = False,
    formats: tuple[str, ...] = ("png",),
) -> Path:
    """读取原始快照，在独占目录生成图像和独立重绘账本。

    Args:
        run_directory: 包含 trace 与 metrics 的单次运行目录。
        output_directory: 新建的输出目录；默认在原 run 的 plots/replots 下创建。
        tactile_detail: 是否请求左右逐 taxel 三轴细节图。
        formats: 显式选择 PNG、PDF 或两者。

    Returns:
        包含实际图像和 rendering_manifest.json 的目录。

    Raises:
        FileNotFoundError: 找不到 trace 或原指标。
        FileExistsError: 指定输出目录已存在，避免覆盖旧图或污染原账本。
        ValueError: 格式、配置或轨迹无效。
    """
    if not formats or any(value not in {"png", "pdf"} for value in formats):
        raise ValueError("输出格式必须为 png 或 pdf。")
    source = Path(run_directory).resolve(strict=True)
    trace_path = source / "trace.parquet"
    if trace_path.is_file():
        import pyarrow.parquet as pq

        rows = pq.read_table(trace_path).to_pylist()
    else:
        trace_path = source / "trace.csv"
        with trace_path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("无法重绘空的力跟踪轨迹。")
    metrics_path = source / "metrics.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    inputs = [trace_path, metrics_path]
    config_path = source / "effective_parameters.json"
    notes = ["曲线来自已存轨迹；降采样损失不可恢复，指标沿用原 metrics.json。"]
    if config_path.is_file():
        config = json.loads(config_path.read_text(encoding="utf-8"))
        inputs.append(config_path)
    else:
        config = {}
        for name in ("profile", "task"):
            path = source / f"{name}.yaml"
            if path.is_file():
                config[name] = yaml.safe_load(path.read_text(encoding="utf-8"))
                inputs.append(path)
        # 旧 profile.yaml 可能是变体覆盖前的输入，不能据此标注本次控制器增益。
        if "profile" in config:
            config.pop("profile")
            notes.append("缺少有效参数快照，省略无法确认的控制器参数标注。")
    if "stiffness_valid" not in rows[0]:
        notes.append("历史轨迹缺少刚度有效性，省略刚度估计面板。")
    output = (
        Path(output_directory).resolve()
        if output_directory is not None
        else source / "plots" / "replots" / f"{datetime.now(UTC):%Y%m%dT%H%M%S}-{uuid4().hex[:8]}"
    )
    output.mkdir(parents=True, exist_ok=False)
    paths = render_run_artifacts(
        trace=rows,
        metrics=metrics,
        config=config,
        output_dir=output,
        tactile_detail=tactile_detail,
        formats=formats,
    )
    if not paths:
        raise ValueError("轨迹缺少可绘制的标准信号。")
    if tactile_detail and not any("tactile_left_detail" in path.stem for path in paths):
        notes.append("左侧触觉细节不可用：缺少完整 3×3 数据或接触确认区间。")
    if tactile_detail and not any("tactile_right_detail" in path.stem for path in paths):
        notes.append("右侧触觉细节不可用：缺少完整 3×3 数据或接触确认区间。")
    manifest = {
        "schema_version": 1,
        "source_run": str(source),
        "created_at": datetime.now(UTC).isoformat(),
        "source_sha256": {path.name: sha256(path.read_bytes()).hexdigest() for path in inputs},
        "artifacts": [str(path.relative_to(output)) for path in paths],
        "formats": list(formats),
        "tactile_detail": tactile_detail,
        "notes": notes,
    }
    (output / "rendering_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return output
