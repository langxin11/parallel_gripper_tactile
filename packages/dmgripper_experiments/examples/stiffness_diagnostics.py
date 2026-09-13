"""读取既有运行目录，打印刚度估计诊断摘要。

示例默认离线：只读取 trace.csv，不接触设备。
"""

from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path


def summarize(directory: Path) -> dict[str, object]:
    """统计运行目录中刚度估计的有效性与更新原因分布。"""
    from dmgripper_experiments.plotting import read_trace_rows

    rows = read_trace_rows(directory)
    reasons = Counter(row.get("stiffness_reason", "") for row in rows)
    valid = [row for row in rows if row.get("stiffness_valid") == "True"]
    updated = [row for row in rows if row.get("stiffness_updated") == "True"]
    values = [float(row["stiffness_n_per_m"]) for row in valid if row.get("stiffness_n_per_m")]
    return {
        "directory": str(directory),
        "samples": len(rows),
        "valid_samples": len(valid),
        "updated_samples": len(updated),
        "reasons": dict(reasons),
        "valid_value_range_n_per_m": ([min(values), max(values)] if values else None),
    }


def main(argv: list[str] | None = None) -> int:
    """打印一个或多个运行目录的刚度诊断摘要。"""
    arguments = list(sys.argv[1:] if argv is None else argv)
    if not arguments:
        print("用法：python stiffness_diagnostics.py <运行目录> [...]", file=sys.stderr)
        return 2
    import json

    for item in arguments:
        print(json.dumps(summarize(Path(item)), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
