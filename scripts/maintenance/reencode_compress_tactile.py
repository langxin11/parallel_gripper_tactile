r"""把旧版 ``tactile.jsonl`` 就地重编码并压缩为 ``tactile.jsonl.gz``。

- 重编码 = 浮点截断到 9 位有效数字（float32 可表示精度）+ 紧凑分隔符，
  与 1.12.0 记录器的输出语义一致；行数与 1000 Hz 记录速率保持不变。
- gzip 压缩；``dmgripper_experiments.replay`` 与
  ``scripts/analysis/replay_pillar_friction.py`` 已支持透明读取 ``.gz``。
- 安全性：逐行解析校验，写入临时文件后全量解压回读核对行数与 MD5，
  通过才原子替换并删除原文件；无法解析的残行原样保留并告警。
- 幂等：已存在 ``tactile.jsonl.gz`` 的目录跳过，可反复执行。

用法::

    python scripts/maintenance/reencode_compress_tactile.py outputs/real

只处理 ``tactile.jsonl``，不触碰 manifest、trace、事件与绘图产物。
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import sys
import time
from pathlib import Path

from dmgripper_experiments.recording import _truncate_float32

_FAILED = "failed"
_SKIPPED = "skipped"
_DONE = "done"


def _encode_line(line: str) -> str | None:
    """把一行旧 JSON 重编码为新格式；残行返回 ``None``。"""
    try:
        record = json.loads(line)
    except json.JSONDecodeError:
        return None
    return (
        json.dumps(
            _truncate_float32(record),
            ensure_ascii=False,
            sort_keys=True,
            allow_nan=False,
            separators=(",", ":"),
        )
        + "\n"
    )


def convert_file(path: Path) -> tuple[str, str]:
    """重编码并压缩单个文件；返回 (状态, 摘要)。

    Raises:
        ValueError: 记录包含新编码无法表达的值（如 NaN）。
    """
    target = path.with_name(path.name + ".gz")
    if target.exists():
        return _SKIPPED, f"{target} 已存在"
    temporary = target.with_name(target.name + ".tmp")
    original_mib = path.stat().st_size / 1048576
    rows = 0
    raw_lines = 0
    digest = hashlib.md5()
    with (
        path.open(encoding="utf-8") as source,
        gzip.open(temporary, "wt", encoding="utf-8", compresslevel=6) as sink,
    ):
        for line in source:
            if not line.strip():
                continue
            payload = _encode_line(line)
            if payload is None:
                payload = line
                raw_lines += 1
            sink.write(payload)
            digest.update(payload.encode("utf-8"))
            rows += 1
    verify_rows = 0
    verify_digest = hashlib.md5()
    with gzip.open(temporary, "rt", encoding="utf-8") as handle:
        for line in handle:
            verify_rows += 1
            verify_digest.update(line.encode("utf-8"))
    if verify_rows != rows or verify_digest.hexdigest() != digest.hexdigest():
        temporary.unlink(missing_ok=True)
        return _FAILED, "解压回读与写入不一致，已保留原文件"
    temporary.replace(target)
    path.unlink()
    size_mib = target.stat().st_size / 1048576
    return _DONE, (
        f"{rows} 行（残行 {raw_lines}），{original_mib:.1f} -> {size_mib:.1f} MiB"
        f"（{original_mib / max(size_mib, 1e-9):.2f}x）"
    )


def main() -> int:
    """批量转换目录树下的 tactile.jsonl 并汇报体积变化。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("roots", nargs="+", type=Path, help="要扫描的输出根目录")
    args = parser.parse_args()
    files = sorted({path for root in args.roots for path in Path(root).rglob("tactile.jsonl")})
    if not files:
        print("未找到 tactile.jsonl。")
        return 0
    total_before = sum(path.stat().st_size for path in files)
    started = time.monotonic()
    failures = 0
    for index, path in enumerate(files, start=1):
        try:
            status, detail = convert_file(path)
        except ValueError as error:
            status, detail = _FAILED, f"记录含非法值：{error}"
        if status == _FAILED:
            failures += 1
        print(f"[{index}/{len(files)}] {status}: {path} — {detail}", flush=True)
    total_after = sum(
        path.with_name(path.name + ".gz").stat().st_size
        for path in files
        if path.with_name(path.name + ".gz").is_file()
    )
    elapsed = time.monotonic() - started
    print(
        f"完成：{total_before / 1048576:.0f} MiB -> {total_after / 1048576:.0f} MiB"
        f"（{total_before / max(total_after, 1):.2f}x），用时 {elapsed:.0f} s；失败 {failures} 个。"
    )
    return 1 if failures else 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
