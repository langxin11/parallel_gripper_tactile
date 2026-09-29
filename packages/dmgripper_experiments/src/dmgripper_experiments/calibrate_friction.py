"""从人工整侧起滑标记前的触觉样本提取条件有效承载比候选。"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

from papillarray_hardware.recording import RECORDING_NAME, iter_records


def _finite_positive(value: object) -> bool:
    return (
        isinstance(value, (float, int))
        and not isinstance(value, bool)
        and math.isfinite(value)
        and value > 0
    )


def calibrate(
    run_dir: Path,
    *,
    window_s: float = 0.2,
    slip_time_s: float | None = None,
    side: str | None = None,
) -> dict[str, object]:
    """计算候选；窗口为可调取样口径，不表示文献保证或材料真 μ。"""
    if not _finite_positive(window_s):
        raise ValueError("window_s 必须为正的有限秒数")
    if (slip_time_s is None) != (side is None):
        raise ValueError("--slip-time-s 与 --side 必须成对给出")
    if side is not None and side not in {"left", "right", "both"}:
        raise ValueError("--side 必须为 left、right 或 both")
    path = Path(run_dir)
    config = json.loads((path / "config.json").read_text(encoding="utf-8"))["effective"]
    if config["stage"] != "friction":
        raise ValueError("仅 friction 阶段的运行可标定")
    recording = path / RECORDING_NAME
    # 只保存连续有效阶段的边界；长时间 250 Hz trace 不整场驻内存。
    segments = []
    current = None
    anchor = None
    for row in iter_records(recording, "/trace"):
        t = row.get("time_s")
        device_us = row.get("tactile_timestamp_us")
        valid = row.get("phase") in {"active", "holding"} and device_us is not None
        if current is not None and (
            not valid or row.get("contact_segment") != current["contact_segment"]
        ):
            current["end_time_s"] = t
            segments.append(current)
            current = None
        if valid:
            if current is None:
                current = {
                    "start_time_s": t,
                    "end_time_s": None,
                    "start_device_us": device_us,
                    "end_device_us": device_us,
                    "contact_segment": row.get("contact_segment"),
                    "last_active_time_s": t,
                }
            current["end_device_us"] = device_us
            current["last_active_time_s"] = t
            if slip_time_s is not None and t <= slip_time_s:
                anchor = row
    if current is not None:
        current["end_time_s"] = (
            current["last_active_time_s"] + 1.0 / config["timing"]["control_rate_hz"]
        )
        segments.append(current)
    if not segments:
        raise ValueError("运行中没有 active／holding 接触段")
    events = list(iter_records(recording, "/events"))
    for segment in segments:
        stopping = [
            e["time_s"]
            for e in events
            if e.get("event") in {"state", "fault"}
            and e.get("phase") not in {"active", "holding"}
            and isinstance(e.get("time_s"), (int, float))
            and e["time_s"] > segment["start_time_s"]
        ]
        if stopping:
            segment["end_time_s"] = min(segment["end_time_s"], min(stopping))
    if slip_time_s is not None:
        if not math.isfinite(slip_time_s):
            raise ValueError("起滑时间必须有限")
        selected_segment = next(
            (
                segment
                for segment in segments
                if segment["start_time_s"] <= slip_time_s < segment["end_time_s"]
            ),
            None,
        )
        if (
            selected_segment is None
            or anchor is None
            or anchor.get("contact_segment") != selected_segment["contact_segment"]
        ):
            raise ValueError("指定起滑时间不在 active／holding 连续接触段内")
        if slip_time_s - anchor["time_s"] > config["timing"]["tactile_timeout_s"]:
            raise ValueError("指定起滑时间距最近触觉观测过远")
        marks = [
            {
                "side": side,
                "time_s": slip_time_s,
                "tactile_timestamp_us": anchor["tactile_timestamp_us"],
                "contact_segment": anchor["contact_segment"],
                "time_adjusted": True,
            }
        ]
    else:
        marks = [
            {**e, "time_adjusted": False} for e in events if e.get("event") == "manual_slip_mark"
        ]
    if not marks:
        raise ValueError("没有人工起滑标记；可指定 --slip-time-s 与 --side")
    result_marks = []
    side_values: dict[str, list[float]] = {"left": [], "right": []}
    prepared = []
    for mark in marks:
        mark_side = mark.get("side")
        if mark_side not in {"left", "right", "both"}:
            continue
        mark_us = int(mark["tactile_timestamp_us"])
        segment = next(
            (
                item
                for item in segments
                if item["contact_segment"] == mark.get("contact_segment")
                and item["start_device_us"] <= mark_us <= item["end_device_us"]
            ),
            None,
        )
        if (
            segment is None
            or not segment["start_time_s"] <= mark.get("time_s", -1) < segment["end_time_s"]
        ):
            raise ValueError("人工标记不在连续有效接触段内")
        first_us = max(int(segment["start_device_us"]), mark_us - round(window_s * 1e6))
        result = {
            "side": mark_side,
            "original_time_s": mark.get("time_s"),
            "used_device_timestamp_us": mark_us,
            "time_adjusted": mark["time_adjusted"],
            "contact_segment": mark.get("contact_segment"),
            "window_start_device_us": first_us,
            "window_end_device_us": mark_us,
            "sides": {},
        }
        prepared.append((first_us, mark_us, mark_side, result, {"left": [], "right": []}))
    if not prepared:
        raise ValueError("人工标记没有匹配的有效接触段")
    # 单次顺序扫描触觉流，只保留各标记前短窗口的样本。
    for sample in iter_records(recording, "/tactile"):
        timestamp = sample.get("timestamp_us")
        if not isinstance(timestamp, int):
            continue
        for first_us, mark_us, mark_side, _result, values_by_side in prepared:
            if not first_us <= timestamp <= mark_us:
                continue
            for selected in ("left", "right") if mark_side == "both" else (mark_side,):
                fx = sample.get(f"raw_{selected}_fx_n")
                fy = sample.get(f"raw_{selected}_fy_n")
                fz = sample.get(f"raw_{selected}_fz_n")
                threshold = config["lifecycle"]["contact_off_n"]
                if not (
                    _finite_positive(fz)
                    and float(fz) > threshold
                    and isinstance(fx, (int, float))
                    and isinstance(fy, (int, float))
                    and math.isfinite(fx)
                    and math.isfinite(fy)
                ):
                    raise ValueError(
                        f"{selected} 起滑前窗口有失接触或无效三轴力；请缩短窗口或重选时间"
                    )
                tangential = math.hypot(fx, fy)
                values_by_side[selected].append(
                    (timestamp, float(fz), tangential, tangential / float(fz))
                )
    for _first_us, _mark_us, mark_side, result, values_by_side in prepared:
        for selected in ("left", "right") if mark_side == "both" else (mark_side,):
            values = values_by_side[selected]
            if not values:
                raise ValueError(f"{selected} 在起滑前窗口没有有效样本")
            ratios = [item[3] for item in values]
            side_values[selected].append(statistics.median(ratios))
            result["sides"][selected] = {
                "candidate_t_over_n": statistics.median(ratios),
                "sample_count": len(values),
                "sample_device_us_min": min(item[0] for item in values),
                "sample_device_us_max": max(item[0] for item in values),
                "normal_n_median": statistics.median(item[1] for item in values),
                "tangential_n_median": statistics.median(item[2] for item in values),
                "ratio_min": min(ratios),
                "ratio_max": max(ratios),
                "ratio_stdev": statistics.pstdev(ratios),
            }
        result_marks.append(result)
    if not result_marks:
        raise ValueError("人工标记没有匹配的有效接触段")
    return {
        "schema": "dmgripper.friction_candidate.v1",
        "meaning": "effective_grasp_condition_candidate_not_material_mu",
        "source_run": str(path.resolve()),
        "object_name": config["metadata"]["object_name"],
        "condition": config["metadata"]["condition"],
        "preload_force_n": config["reference"]["initial_force_n"],
        "preload_source": config["reference"]["preload_source"],
        "window_s": window_s,
        "markers": result_marks,
        "left": None
        if not side_values["left"]
        else {
            "candidate_t_over_n": statistics.median(side_values["left"]),
            "marker_count": len(side_values["left"]),
        },
        "right": None
        if not side_values["right"]
        else {
            "candidate_t_over_n": statistics.median(side_values["right"]),
            "marker_count": len(side_values["right"]),
        },
        "acceptance": "manual_review_required",
    }


def main() -> None:
    """读取 MCAP 并独占写出候选 JSON，绝不改动运行配置。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="friction 阶段运行目录")
    parser.add_argument("--window-s", type=float, default=0.2, help="起滑前取样窗口秒数，可调口径")
    parser.add_argument("--slip-time-s", type=float, help="人工修正的运行相对起滑秒数")
    parser.add_argument(
        "--side", choices=("left", "right", "both"), help="修正时间对应的整侧滑动观察"
    )
    parser.add_argument(
        "--output", type=Path, help="候选 JSON 路径；默认 RUN_DIR/friction_candidate.json"
    )
    args = parser.parse_args()
    candidate = calibrate(
        args.run_dir, window_s=args.window_s, slip_time_s=args.slip_time_s, side=args.side
    )
    output = args.output or args.run_dir / "friction_candidate.json"
    with output.open("x", encoding="utf-8") as handle:
        json.dump(candidate, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    print(output)


if __name__ == "__main__":
    main()
