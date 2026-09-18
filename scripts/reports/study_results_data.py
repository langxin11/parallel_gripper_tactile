"""把 `reports/combined.typ`（研究证据汇总部分）的字面量数据块从 study 产物重新生成。

合集报告定位为时点性交付物：正文与表格里的数字直接写在报告文件里，编译不读取
`outputs/`，因此产物被清理后报告仍然可读。本脚本负责在换 run 或重跑研究后刷新
那个数据块，并在需要时校验已提交的数据块是否与产物一致。

用法：

```bash
uv run python scripts/reports/study_results_data.py            # 重新生成数据块
uv run python scripts/reports/study_results_data.py --check    # 只校验，不写文件
```
"""

from __future__ import annotations

import argparse
import csv
import json
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
REPORT = ROOT / "reports" / "combined.typ"
BEGIN_MARKER = "// ==== 数据块开始（由 scripts/reports/study_results_data.py 生成，勿手改） ===="
END_MARKER = "// ==== 数据块结束 ===="

# 报告引用的 run：每项研究固定一个已完成 run 的产物目录（相对仓库根）。
STUDIES = {
    "controller": "outputs/studies/force_tracking_controller_comparison/20260908T165718Z-3dc74f12",
    "ablation": "outputs/studies/force_tracking_ablation/20260904T080538Z-d843599a",
    "adrc_coarse": "outputs/studies/force_tracking_torque_adrc_tuning/coarse/20260904T080731Z-87b82464",
    "adrc_confirm": "outputs/studies/force_tracking_torque_adrc_tuning/confirm/20260904T081426Z-9940bbad",
    "estimator": "outputs/studies/force_tracking_stiffness_estimator_comparison/20260904T080538Z-49cb698e",
    "admittance": "outputs/research/studies/dm_admittance_tuning/20260918T113544008350",
    "robotiq": "outputs/studies/robotiq_discrete_force/20260906T123718Z-05c7bc01",
    "friction": "outputs/studies/friction_estimation_local_slip/20260905T125708Z-4b9e154b",
}

TASK_FIELDS = {
    "step_force_tracking": "step",
    "ramp_force_tracking": "ramp",
    "mixed_waypoint_force_tracking": "mixed",
}
TASK_LABELS = {
    "step_force_tracking": "Step",
    "ramp_force_tracking": "Ramp",
    "mixed_waypoint_force_tracking": "Mixed",
}
FRICTION_SCENARIOS = {
    "low_friction_probe": "低摩擦",
    "nominal_friction_probe": "名义",
    "high_friction_probe": "高摩擦",
    "noisy_friction_probe": "含噪",
    "no_slip_low_probe": "负对照",
}
# 变体名到数据块键名：Typst 字段访问保留标识符，避免出现连字符歧义。
RATIO_KEYS = {
    "pid-only": "pid_only",
    "pid-torque-ff": "pid_torque_ff",
    "pid-stiffness-ff": "pid_stiffness_ff",
    "pid-stiffness-limit": "pid_stiffness_limit",
    "full": "full",
    "adrc-torque": "adrc_torque",
}


class ReportDataError(RuntimeError):
    """产物缺失或字段异常时抛出。"""


def _read_rows(path: Path) -> list[dict[str, str]]:
    """读取 CSV 产物为「列名 → 字符串」记录数组。"""
    if not path.is_file():
        raise ReportDataError(f"缺少产物文件：{path}")
    with path.open(encoding="utf-8", newline="") as handle:
        return [{key: value for key, value in row.items()} for row in csv.DictReader(handle)]


def _number(value: str | None) -> float | None:
    """空串与非数值返回 ``None``。"""
    if value is None or value == "":
        return None
    return float(value)


def _mean(rows: Sequence[dict[str, str]], key: str) -> float:
    """数值列均值；空值跳过。"""
    values = [number for row in rows if (number := _number(row.get(key))) is not None]
    if not values:
        raise ReportDataError(f"列 {key} 没有有效数值")
    return sum(values) / len(values)


def _maximum(rows: Sequence[dict[str, str]], key: str) -> float:
    """数值列最大值。"""
    values = [number for row in rows if (number := _number(row.get(key))) is not None]
    if not values:
        raise ReportDataError(f"列 {key} 没有有效数值")
    return max(values)


def _count_true(rows: Sequence[dict[str, str]], key: str) -> int:
    """统计取值为 ``True``（大小写不敏感）的行数。"""
    return sum(1 for row in rows if row.get(key, "").lower() == "true")


def _group(rows: Iterable[dict[str, str]], key: str) -> dict[str, list[dict[str, str]]]:
    """按字段值分组，保留首次出现顺序。"""
    groups: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        groups.setdefault(row[key], []).append(row)
    return groups


def _task_mean(rows: Sequence[dict[str, str]], task: str) -> float:
    """某个目标任务下的 RMSE 均值。"""
    return _mean([row for row in rows if row["task_name"] == task], "rmse_n")


def _controller(root: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """控制器选型：逐变体聚合与正文需要的派生量。"""
    records = _read_rows(root / STUDIES["controller"] / "summary.csv")
    groups = _group(records, "controller_variant")
    rows: list[dict[str, Any]] = []
    ratios: dict[str, dict[str, float]] = {}
    rmse_by_variant: dict[str, float] = {}
    for variant, members in groups.items():
        rmse_by_variant[variant] = _mean(members, "rmse_n")
    for variant, members in groups.items():
        rows.append(
            {
                "variant": variant,
                "step": _task_mean(members, "step_force_tracking"),
                "ramp": _task_mean(members, "ramp_force_tracking"),
                "mixed": _task_mean(members, "mixed_waypoint_force_tracking"),
                "rmse": rmse_by_variant[variant],
                "mae": _mean(members, "mae_n"),
                "overshoot": _mean(members, "overshoot_ratio"),
                "passed": f"{_count_true(members, 'passed')}/{len(members)}",
            }
        )
        ratios[RATIO_KEYS[variant]] = {
            field: _task_mean(members, task) / _task_mean(groups["full"], task)
            for task, field in TASK_FIELDS.items()
        }
    rows.sort(key=lambda row: row["rmse"])
    stats = {
        "conditions": len(records),
        "max_output_saturation": max(
            _maximum(records, "torque_saturation_ratio"),
            _maximum(records, "position_saturation_ratio"),
        ),
        "delta_torque_ff": 1 - rmse_by_variant["pid-torque-ff"] / rmse_by_variant["pid-only"],
        "delta_stiffness_ff": 1 - rmse_by_variant["pid-stiffness-ff"] / rmse_by_variant["pid-only"],
        "ratios": ratios,
    }
    return rows, stats


def _ablation(root: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """PID 2×2 消融：逐变体逐材料 RMSE 与两个主效应。"""
    records = _read_rows(root / STUDIES["ablation"] / "summary.csv")
    groups = _group(records, "controller_variant")
    rows: list[dict[str, Any]] = []
    overall: dict[str, float] = {}
    for variant, members in groups.items():
        overall[variant] = _mean(members, "rmse_n")
        by_material = _group(members, "object_material")
        rows.append(
            {
                "variant": variant,
                "medium": _mean(by_material["medium"], "rmse_n"),
                "hard": _mean(by_material["hard"], "rmse_n"),
                "stiff": _mean(by_material["stiff"], "rmse_n"),
                "all": overall[variant],
            }
        )
    rows.sort(key=lambda row: row["all"])
    stats = {
        "conditions": len(records),
        "delta_torque_ff": 1 - overall["full"] / overall["pid-only"],
        "delta_stiffness_ff": 1 - overall["pid-stiffness-ff"] / overall["pid-only"],
    }
    return rows, stats


def _adrc(
    root: Path,
) -> tuple[list[dict[str, Any]], dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    """Torque ADRC：coarse 排名前五与 confirm 逐条件聚合。"""
    ranking = _read_rows(root / STUDIES["adrc_coarse"] / "candidate_ranking.csv")
    ranking.sort(key=lambda row: int(row["rank"]))
    feasible = [row for row in ranking if row["feasible"].lower() == "true"]
    coarse_rows = [
        {
            "rank": int(row["rank"]),
            "fc": _number(row["measurement_filter_cutoff_hz"]),
            "wc": _number(row["controller_bandwidth_rad_s"]),
            "ratio": _number(row["observer_bandwidth_ratio"]),
            "feasible": row["feasible"].lower() == "true",
            "overshoot": _number(row["step_overshoot_ratio_mean"]),
            "ramp_ratio": _number(row["ramp_rmse_ratio_to_baseline"]),
            "mixed_ratio": _number(row["mixed_rmse_ratio_to_baseline"]),
        }
        for row in ranking[:5]
    ]
    coarse_stats = {
        "conditions": len(_read_rows(root / STUDIES["adrc_coarse"] / "summary.csv")),
        "candidates": len(ranking),
        "feasible_count": len(feasible),
        "feasible_id": feasible[0]["candidate_id"] if feasible else "无",
        "max_saturation": _maximum(ranking, "max_torque_saturation_ratio"),
    }

    summary = _read_rows(root / STUDIES["adrc_confirm"] / "summary.csv")
    aggregate = _read_rows(root / STUDIES["adrc_confirm"] / "aggregate.csv")
    confirm_rows = [
        {
            "task": TASK_LABELS[row["task_name"]],
            "material": row["object_material"],
            "rmse": _number(row["rmse_n_mean"]),
            "std": _number(row["rmse_n_std"]),
            "overshoot": _number(row["overshoot_ratio_mean"]),
            "saturation": _number(row["torque_saturation_ratio_mean"]),
        }
        for row in aggregate
    ]
    winner = _read_rows(root / STUDIES["adrc_confirm"] / "candidate_ranking.csv")[0]
    confirm_stats = {
        "conditions": len(summary),
        "winner_id": winner["candidate_id"],
        "step_overshoot": _number(winner["step_overshoot_ratio_mean"]),
    }
    return coarse_rows, coarse_stats, confirm_rows, confirm_stats


def _estimator(root: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """刚度估计器对比：逐估计器 RMSE、相对 secant 的比值与平均等效刚度。"""
    records = _read_rows(root / STUDIES["estimator"] / "summary.csv")
    groups = _group(records, "stiffness_estimator_method")
    baseline = _mean(groups["secant_ewma"], "rmse_n")
    rows: list[dict[str, Any]] = []
    for estimator, members in groups.items():
        rows.append(
            {
                "estimator": estimator,
                "step": _task_mean(members, "step_force_tracking"),
                "ramp": _task_mean(members, "ramp_force_tracking"),
                "mixed": _task_mean(members, "mixed_waypoint_force_tracking"),
                "rmse": _mean(members, "rmse_n"),
                "relative": _mean(members, "rmse_n") / baseline,
                "stiffness": _mean(members, "mean_estimated_stiffness_n_per_m"),
            }
        )
    rows.sort(key=lambda row: row["rmse"])
    rmse_values = [row["rmse"] for row in rows]
    stiffness_values = [row["stiffness"] for row in rows]
    stats = {
        "conditions": len(records),
        "rmse_gap": max(rmse_values) / min(rmse_values) - 1,
        "stiffness_min": min(stiffness_values),
        "stiffness_max": max(stiffness_values),
        "stiffness_gap": max(stiffness_values) / min(stiffness_values) - 1,
    }
    return rows, stats


def _admittance(root: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """DM 导纳整定：排名前五候选与榜首参数。"""
    ranking = _read_rows(root / STUDIES["admittance"] / "candidate_ranking.csv")
    ranking.sort(key=lambda row: int(row["rank"]))
    rows = [
        {
            "rank": int(row["rank"]),
            "cutoff": _number(row["filter_cutoff_hz"]),
            "velocity": _number(row["velocity_limit_rad_s"]),
            "feedforward": _number(row["approach_feedforward_force_n"]),
            "rmse": _number(row["rmse_n_mean"]),
        }
        for row in ranking[:5]
    ]
    best = ranking[0]
    stats = {
        "conditions": len(_read_rows(root / STUDIES["admittance"] / "summary.csv")),
        "candidates": len(ranking),
        "stable_count": _count_true(ranking, "stable"),
        "mass": _number(best["mass_kg"]),
        "damping": _number(best["damping_ns_m"]),
        "stiffness": _number(best["stiffness_n_m"]),
        "velocity": _number(best["velocity_limit_rad_s"]),
        "feedforward": _number(best["approach_feedforward_force_n"]),
        "rmse": _number(best["rmse_n_mean"]),
    }
    return rows, stats


def _robotiq(root: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Robotiq 离散力：五变体聚合与正文引用的对照量。"""
    records = _read_rows(root / STUDIES["robotiq"] / "summary.csv")
    aggregate = _read_rows(root / STUDIES["robotiq"] / "aggregate.csv")
    rows = [
        {
            "variant": row["controller_variant"],
            "passed": int(row["passed_runs"]),
            "rmse": _number(row["rmse_n_mean"]),
            "hold": _number(row["hold_ratio_mean"]),
            "steady_error": _number(row["steady_force_error_n_mean"]),
        }
        for row in aggregate
    ]
    by_variant = {row["controller_variant"]: row for row in aggregate}
    stats = {
        "conditions": len(records),
        "dynamic_step_hold": _number(by_variant["dynamic-step"]["hold_ratio_mean"]),
        "dynamic_step_actions": _number(by_variant["dynamic-step"]["action_count_mean"]),
        "dynamic_step_steady_error": _number(
            by_variant["dynamic-step"]["steady_force_error_n_mean"]
        ),
        "fixed_step_steady_error": _number(by_variant["fixed-step"]["steady_force_error_n_mean"]),
        "quantized_pi_reverse": _number(by_variant["quantized-pi"]["reverse_count_mean"]),
    }
    return rows, stats


def _friction(root: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """摩擦局部起滑：逐场景聚合与验收计数。"""
    records = _read_rows(root / STUDIES["friction"] / "summary.csv")
    aggregate = _read_rows(root / STUDIES["friction"] / "aggregate.csv")
    rows = [
        {
            "scenario": FRICTION_SCENARIOS[row["scenario"]],
            "expect_local_slip": row["expect_local_slip"].lower() == "true",
            "detection_rate": _number(row["detection_rate"]),
            "false_positive_rate": _number(row["false_positive_rate"]),
            "detection_lead": _number(row["detection_lead_s_mean"]),
            "passed": int(row["passed_runs"]),
        }
        for row in aggregate
    ]
    negative = [row for row in records if row["expect_local_slip"].lower() != "true"]
    stats = {
        "expected": _count_true(records, "expect_local_slip"),
        "detected": _count_true(records, "local_slip_detected"),
        "negative": len(negative),
        "false_positive": sum(
            1 for row in negative if row["local_slip_detected"].lower() == "true"
        ),
        "validation_passed": _count_true(records, "validation_passed"),
        "control_qualified": _count_true(records, "control_candidate_qualified"),
    }
    return rows, stats


def _typst(value: Any) -> str:
    """把 Python 值渲染成 Typst 字面量。"""
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "none"
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        text = f"{value:.6g}"
        return text if ("." in text or "e" in text or "E" in text) else text + ".0"
    if isinstance(value, dict):
        return "(" + ", ".join(f"{key}: {_typst(item)}" for key, item in value.items()) + ")"
    raise TypeError(f"不支持的取值类型：{type(value)!r}")


def _rows_literal(rows: Sequence[dict[str, Any]]) -> str:
    """把记录数组渲染成多行 Typst 数组，一行一条记录便于审阅。"""
    lines = ["("]
    for row in rows:
        fields = ", ".join(f"{key}: {_typst(value)}" for key, value in row.items())
        lines.append(f"  ({fields}),")
    lines.append(")")
    return "\n".join(lines)


def _definition(name: str, rows: Sequence[dict[str, Any]]) -> str:
    """渲染 `#let <name> = (…)` 定义。"""
    return f"#let {name} = " + _rows_literal(rows)


def _dict_definition(name: str, values: dict[str, Any]) -> str:
    """渲染多行 `#let <name> = (…)` 映射定义。"""
    lines = [f"#let {name} = ("]
    for key, value in values.items():
        lines.append(f"  {key}: {_typst(value)},")
    lines.append(")")
    return "\n".join(lines)


def build_block(root: Path) -> str:
    """按当前产物生成完整数据块（不含标记行）。"""
    controller_rows, controller_stats = _controller(root)
    ablation_rows, ablation_stats = _ablation(root)
    coarse_rows, coarse_stats, confirm_rows, confirm_stats = _adrc(root)
    estimator_rows, estimator_stats = _estimator(root)
    admittance_rows, admittance_stats = _admittance(root)
    robotiq_rows, robotiq_stats = _robotiq(root)
    friction_rows, friction_stats = _friction(root)

    sections = [
        "// 报告数据全部字面写在本块内：编译不读取 outputs/，产物被清理后报告仍然可读。",
        "// 换 run 后运行 `uv run python scripts/reports/study_results_data.py` 重新生成本块；",
        "// `--check` 校验本块与产物一致；运行级溯源保留在研究产物中，不进入科研叙事。",
        _definition("controller-rows", controller_rows),
        _dict_definition("controller-stats", controller_stats),
        "",
        _definition("ablation-rows", ablation_rows),
        _dict_definition("ablation-stats", ablation_stats),
        "",
        _definition("coarse-rows", coarse_rows),
        _dict_definition("coarse-stats", coarse_stats),
        _definition("confirm-rows", confirm_rows),
        _dict_definition("confirm-stats", confirm_stats),
        "",
        _definition("estimator-rows", estimator_rows),
        _dict_definition("estimator-stats", estimator_stats),
        "",
        _definition("admittance-rows", admittance_rows),
        _dict_definition("admittance-stats", admittance_stats),
        "",
        _definition("robotiq-rows", robotiq_rows),
        _dict_definition("robotiq-stats", robotiq_stats),
        "",
        _definition("friction-rows", friction_rows),
        _dict_definition("friction-stats", friction_stats),
    ]
    return "\n".join(sections)


def _block_bounds(text: str) -> tuple[int, int]:
    """返回数据块内容的起止下标。"""
    if BEGIN_MARKER not in text or END_MARKER not in text:
        raise ReportDataError(f"{REPORT.name} 缺少数据块标记行")
    start = text.index(BEGIN_MARKER) + len(BEGIN_MARKER)
    end = text.index(END_MARKER)
    return start, end


def updated_report(root: Path, report: Path) -> str:
    """返回写入新数据块后的报告全文。"""
    text = report.read_text(encoding="utf-8")
    start, end = _block_bounds(text)
    return text[:start] + "\n" + build_block(root) + "\n" + text[end:]


def main(argv: Sequence[str] | None = None) -> int:
    """生成或校验报告数据块。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT, help="仓库根目录。")
    parser.add_argument("--report", type=Path, default=REPORT, help="报告文件路径。")
    parser.add_argument("--check", action="store_true", help="只校验数据块是否为最新。")
    args = parser.parse_args(argv)

    root = args.root.resolve()
    report = args.report if args.report.is_absolute() else root / args.report
    try:
        updated = updated_report(root, report)
    except ReportDataError as error:
        print(f"无法生成数据块：{error}")
        return 1
    current = report.read_text(encoding="utf-8")
    if args.check:
        if updated == current:
            print(f"数据块与产物一致：{report.relative_to(root)}")
            return 0
        print(
            f"数据块已过期，请运行 scripts/reports/study_results_data.py：{report.relative_to(root)}"
        )
        return 1
    report.write_text(updated, encoding="utf-8")
    print(f"已刷新数据块：{report.relative_to(root)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
