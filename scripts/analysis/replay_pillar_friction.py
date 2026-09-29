"""在已记录的触觉流上离线重放自主逐 pillar 摩擦估计，并与在线策略的摩擦状态对比。

仓库里有两条互不相干的摩擦估计路径：

1. 抓取回路：``dm_grasp_core.tactile.risk.TaxelRiskObserver`` 产出候选，
   ``grasp.unified`` 消费为分侧摩擦状态，写进 ``trace.csv`` 的 ``adaptive_*_mu``。
2. 独立记录器：``papillarray_hardware.pillar_friction.PillarFrictionEstimator``
   在 ``StandaloneSlipSession`` 的稳定接触门禁后启动，写 ``own_friction_estimate`` 事件。

路径 2 从未在真机上跑过（``outputs/real/pillar-friction/`` 为空），因此本脚本把
路径 2 重放到路径 1 已有的 ``tactile.jsonl`` 上，用**同一份触觉数据**回答：换一个
估计量，会得出什么数。

重放刻意直接复用 ``StandaloneSlipSession`` 而不是重写门禁，保证接触滞回、稳定窗口、
失鲜与断包判据与真机路径逐行一致；``now_s`` 传快照自身的 ``received_at_s``，等价于
"此刻刚收到"。读取 ``session._reference_mask`` 等私有状态仅用于诊断输出。

脚本提供两种模式，用来把"门禁卡住"与"估计器算不出"分开：

``session``
    完全照真机路径走 ``StandaloneSlipSession`` 的稳定接触门禁。
``forced``
    跳过门禁，直接在指定相位起点用当时的接触集合启动 ``PillarFrictionEstimator``，
    回答"若门禁不是障碍，估计器在这段数据上会给出什么"。

用法：

```bash
uv run python scripts/analysis/replay_pillar_friction.py
uv run python scripts/analysis/replay_pillar_friction.py --mode forced --phase active
uv run python scripts/analysis/replay_pillar_friction.py --run <产物目录> --json out.json
```
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from papillarray_hardware.acquisition import TactileSnapshot
from papillarray_hardware.pillar_friction import PillarFrictionEstimator
from papillarray_hardware.standalone import StandaloneSlipConfig, StandaloneSlipSession

TACTILE_NAMES = ("tactile.jsonl", "tactile.jsonl.gz")


def find_tactile(run_directory: Path) -> Path | None:
    """返回运行目录下的触觉记录，优先未压缩版本。"""
    for name in TACTILE_NAMES:
        candidate = run_directory / name
        if candidate.is_file():
            return candidate
    return None


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RUN_ROOT = ROOT / "outputs" / "real" / "online-friction-adaptive" / "bluetooth-earbud-case"
# 与 StandaloneSlipConfig 默认一致；重放时显式写出，避免默认值变化悄悄改变结论。
SESSION = StandaloneSlipConfig(native_slip_enabled=False, own_friction_enabled=True)
# forced 模式复现 standalone._contact_mask 的滞回阈值。
CONTACT_ON_N = SESSION.contact_on_n
CONTACT_OFF_N = SESSION.contact_off_n


@dataclass(frozen=True, slots=True)
class _StubStatus:
    """替代原厂滑移服务状态；重放不使用原厂路径。"""

    session_id: int = 0
    phase: str = "idle"
    reason: str = "not_requested"


@dataclass
class _StubWorker:
    """``StandaloneSlipSession`` 要求的最小 worker 桩。"""

    native_slip_status: _StubStatus = field(default_factory=_StubStatus)
    stops: list[str] = field(default_factory=list)

    def stop_native_slip(self, reason: str) -> None:
        """记录被要求停止原厂会话的原因；重放中原厂路径未启用。"""
        self.stops.append(reason)


@dataclass(frozen=True, slots=True)
class ReplayOutcome:
    """一次重放的会话门禁与逐点估计结果。"""

    run: str
    mode: str
    snapshots: int
    started: bool
    start_time_s: float | None
    start_masks: tuple[tuple[bool, ...], ...]
    stop_reason: str | None
    estimates: tuple[dict[str, Any], ...]
    stop_time_s: float | None = None
    # 估计器内部状态：解锁（比值涨过门槛）与检出（候选确认）分别卡在哪一步。
    armed: tuple[tuple[bool, ...], ...] = ()
    detected: tuple[tuple[bool, ...], ...] = ()

    @property
    def sides(self) -> dict[str, int]:
        """按侧统计产生的估计数量。"""
        return {
            "left": sum(1 for item in self.estimates if item["side"] == "left"),
            "right": sum(1 for item in self.estimates if item["side"] == "right"),
        }


def iter_snapshots(path: Path) -> Any:
    """把 ``tactile.jsonl`` 逐行还原为 ``TactileSnapshot``，供会话门禁消费。"""
    opener = gzip.open if path.suffix == ".gz" else Path.open
    with opener(path, mode="rt", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            record = json.loads(line)
            left = record.get("left_taxel_forces_n")
            right = record.get("right_taxel_forces_n")
            if not left or not right:
                continue
            yield TactileSnapshot(
                received_at_s=float(record["received_at_s"]),
                packet_counter=int(record["packet_counter"]),
                timestamp_us=int(record["timestamp_us"]),
                left_force_n=float(record["left_force_n"]),
                right_force_n=float(record["right_force_n"]),
                raw_left_fz_n=float(record["raw_left_fz_n"]),
                raw_right_fz_n=float(record["raw_right_fz_n"]),
                left_taxel_forces_n=tuple(tuple(row) for row in left),
                right_taxel_forces_n=tuple(tuple(row) for row in right),
                counter_event=record.get("counter_event", "ok"),
                counter_gap=record.get("counter_gap"),
            )


PHASE_ORDER = (
    "approach",
    "contact_transition",
    "preload",
    "active",
    "holding",
    "returning",
)


def phase_ranges(run_directory: Path) -> list[tuple[str, float, float]]:
    """从 trace.csv 汇总各相位的触觉设备时间区间，供估计时刻归位。

    独立记录器不写 ``trace.csv``（它不跑抓取生命周期），此时返回空表，
    由调用方退化为按设备时间直接报告而不做相位归位。
    """
    if not (run_directory / "trace.csv").exists():
        return []
    spans: dict[str, list[float]] = {}
    with (run_directory / "trace.csv").open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            raw = row.get("tactile_timestamp_us") or ""
            if raw:
                spans.setdefault(row["phase"], []).append(float(raw) * 1e-6)
    return [(name, min(spans[name]), max(spans[name])) for name in PHASE_ORDER if name in spans]


def locate(spans: list[tuple[str, float, float]], stamp_s: float) -> str:
    """把一个触觉设备时刻归入相位；无 trace.csv 时显式说明相位不可用。"""
    if not spans:
        return "无相位信息"
    for name, low, high in spans:
        if low <= stamp_s <= high:
            return name
    return "区间外"


def phase_window(run_directory: Path, phase: str) -> tuple[float, float] | None:
    """从 trace.csv 取指定相位的触觉设备时间范围。"""
    for name, low, high in phase_ranges(run_directory):
        if name == phase:
            return low, high
    return None


def hysteresis_mask(
    forces: tuple[Any, Any], previous: tuple[tuple[bool, ...], ...]
) -> tuple[tuple[bool, ...], ...]:
    """复现 ``StandaloneSlipSession._contact_mask`` 的逐触点法向力滞回。"""
    return tuple(
        tuple(
            bool(row[2] >= (CONTACT_OFF_N if previous and previous[side][index] else CONTACT_ON_N))
            for index, row in enumerate(side_forces)
        )
        for side, side_forces in enumerate(forces)
    )


def _as_estimate(
    side: int, item: tuple[int, float, float, float, float], sample: TactileSnapshot
) -> dict[str, Any]:
    """把估计器返回的元组整理成与事件流一致的诊断字典。"""
    pillar_id, raw_mu, conservative_mu, normal, shear = item
    return {
        "event": "own_friction_estimate",
        "side": "left" if side == 0 else "right",
        "pillar_id": pillar_id,
        "raw_mu": raw_mu,
        "conservative_mu": conservative_mu,
        "normal_force_n": normal,
        "shear_force_n": shear,
        "device_timestamp_us": sample.timestamp_us,
        "packet_counter": sample.packet_counter,
    }


def _estimator_state(estimator: PillarFrictionEstimator) -> tuple[Any, Any]:
    """读出估计器内部逐 pillar 的解锁／检出状态；停止后侧对象已释放则返回空。"""
    sides = getattr(estimator, "_sides", None)
    if sides is None:
        return (), ()
    return (
        tuple(tuple(bool(v) for v in side.armed) for side in sides),
        tuple(tuple(bool(v) for v in side.detected) for side in sides),
    )


def replay(run_directory: Path) -> ReplayOutcome:
    """照真机路径走 ``StandaloneSlipSession`` 门禁重放路径 2。"""
    session = StandaloneSlipSession(SESSION)
    worker = _StubWorker()
    estimates: list[dict[str, Any]] = []
    snapshots = 0
    start_time: float | None = None
    stop_reason: str | None = None
    stop_time_s: float | None = None
    armed: tuple[tuple[bool, ...], ...] = ()
    detected: tuple[tuple[bool, ...], ...] = ()
    tactile_path = find_tactile(run_directory)
    assert tactile_path is not None, f"{run_directory} 下没有触觉记录"
    for sample in iter_snapshots(tactile_path):
        snapshots += 1
        for event in session.update(sample, now_s=sample.received_at_s, worker=worker):
            name = event.get("event")
            if name == "own_friction_state":
                # 同一相位字段区分启动与停止；两者都会带 device_timestamp_us。
                if event.get("phase") == "active" and start_time is None:
                    start_time = float(event["device_timestamp_us"]) * 1e-6
                elif event.get("phase") == "stopped":
                    stop_reason = str(event.get("reason", "stopped"))
                    stop_time_s = float(event["device_timestamp_us"]) * 1e-6
            elif name == "own_friction_estimate":
                estimates.append(dict(event))
        state = _estimator_state(session._own_estimator)
        if state[0]:
            armed, detected = state
    reference = getattr(session, "_reference_mask", ())
    return ReplayOutcome(
        run=run_directory.name,
        mode="session",
        snapshots=snapshots,
        started=start_time is not None,
        start_time_s=start_time,
        start_masks=tuple(tuple(bool(v) for v in side) for side in reference),
        stop_reason=stop_reason,
        stop_time_s=stop_time_s,
        estimates=tuple(estimates),
        armed=armed,
        detected=detected,
    )


def replay_forced(run_directory: Path, *, phase: str) -> ReplayOutcome:
    """跳过门禁，在指定相位起点用当时的接触集合直接启动估计器。"""
    window = phase_window(run_directory, phase)
    if window is None:
        raise SystemExit(f"{run_directory.name} 的 trace.csv 中没有相位 {phase}")
    estimator = PillarFrictionEstimator(SESSION.own_friction)
    estimates: list[dict[str, Any]] = []
    snapshots = 0
    started = False
    start_masks: tuple[tuple[bool, ...], ...] = ()
    previous: tuple[tuple[bool, ...], ...] = ()
    tactile_path = find_tactile(run_directory)
    assert tactile_path is not None, f"{run_directory} 下没有触觉记录"
    snapshot_iter = iter_snapshots(tactile_path)
    for sample in snapshot_iter:
        if sample.timestamp_us * 1e-6 < window[0]:
            continue
        snapshots += 1
        forces = (sample.left_taxel_forces_n, sample.right_taxel_forces_n)
        previous = hysteresis_mask(forces, previous)
        if not started:
            if not all(any(side) for side in previous):
                continue
            estimator.start(previous)
            start_masks, started = previous, True
            continue
        for side, item in enumerate(_group(estimator.update(sample, previous))):
            estimates.extend(_as_estimate(side, entry, sample) for entry in item)
    armed, detected = _estimator_state(estimator)
    if started:
        estimator.stop()
    return ReplayOutcome(
        run=run_directory.name,
        mode=f"forced:{phase}",
        snapshots=snapshots,
        started=started,
        start_time_s=window[0] if started else None,
        start_masks=start_masks,
        stop_reason=None,
        estimates=tuple(estimates),
        armed=armed,
        detected=detected,
    )


def _group(estimates: list[Any]) -> tuple[list[tuple[int, float, float, float, float]], ...]:
    """把估计器返回的扁平列表按侧拆分，便于与事件流对齐。"""
    grouped: tuple[list[tuple[int, float, float, float, float]], ...] = ([], [])
    for item in estimates:
        grouped[item.side].append(
            (
                item.pillar_id,
                item.raw_mu,
                item.conservative_mu,
                item.normal_force_n,
                item.shear_force_n,
            )
        )
    return grouped


def online_friction(run_directory: Path) -> dict[str, tuple[float, float, int]]:
    """读取路径 1 实际写入 trace 的摩擦状态与候选区间；无 trace 时返回空表。"""
    if not (run_directory / "trace.csv").exists():
        return {}
    columns = {
        "left": ("adaptive_left_mu", "adaptive_left_candidate"),
        "right": ("adaptive_right_mu", "adaptive_right_candidate"),
    }
    gathered: dict[str, dict[str, list[float]]] = {
        side: {"mu": [], "candidate": []} for side in columns
    }
    with (run_directory / "trace.csv").open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            for side, (mu_key, candidate_key) in columns.items():
                for target, key in (("mu", mu_key), ("candidate", candidate_key)):
                    raw = row.get(key) or ""
                    if raw:
                        gathered[side][target].append(float(raw))
    summary: dict[str, tuple[float, float, int]] = {}
    for side, values in gathered.items():
        mu = values["mu"]
        candidates = values["candidate"]
        if not mu:
            continue
        summary[side] = (
            min(mu),
            max(mu),
            len({round(value, 6) for value in candidates}),
        )
    return summary


def report(
    outcome: ReplayOutcome,
    online: dict[str, tuple[float, float, int]],
    spans: list[tuple[str, float, float]],
) -> None:
    """打印一次重放的门禁结果、逐点估计与路径 1 的对照。"""
    print(f"\n{'=' * 72}\n{outcome.run}（{outcome.snapshots} 个快照，模式 {outcome.mode}）")
    if not outcome.started:
        print("  路径 2 未启动：稳定接触门禁未通过（或中途失鲜／断包）")
        if outcome.stop_reason:
            print(f"  停止原因：{outcome.stop_reason}")
        return
    left_mask = "".join("1" if v else "0" for v in outcome.start_masks[0])
    right_mask = "".join("1" if v else "0" for v in outcome.start_masks[1])
    print(f"  路径 2 已启动 @ 设备时间 {outcome.start_time_s:.3f} s")
    print(
        f"  冻结参考触点：左 {left_mask}（{sum(outcome.start_masks[0])} 个）"
        f"  右 {right_mask}（{sum(outcome.start_masks[1])} 个）"
    )
    if outcome.stop_reason:
        stamp = outcome.stop_time_s
        where = "" if stamp is None else f" @ {stamp:.3f} s [{locate(spans, stamp)}]"
        print(f"  会话停止：{outcome.stop_reason}{where}")
    if outcome.armed:
        for index, label in enumerate(("左", "右")):
            print(
                f"  {label}解锁状态 armed   = "
                f"{''.join('1' if v else '0' for v in outcome.armed[index])}"
                f"   检出 detected = "
                f"{''.join('1' if v else '0' for v in outcome.detected[index])}"
            )
    sides = outcome.sides
    print(f"  产生估计：左 {sides['left']} 条，右 {sides['right']} 条")
    for item in outcome.estimates:
        stamp = float(item["device_timestamp_us"]) * 1e-6
        print(
            f"    {item['side']:<5} pillar {item['pillar_id']}  "
            f"raw_mu={item['raw_mu']:.4f}  conservative_mu={item['conservative_mu']:.4f}"
            f"  @Fn={item['normal_force_n']:.3f} N Ft={item['shear_force_n']:.3f} N"
            f"  [{locate(spans, stamp)} {stamp:.2f}s]"
        )
    if not online:
        print("  路径 1 对照：无 trace.csv，本次记录不含抓取回路的摩擦状态")
    for side, (low, high, candidate_count) in sorted(online.items()):
        if low == high:
            span = f"{low:.4f}（全程未变）"
        else:
            span = f"[{low:.4f}, {high:.4f}]"
        print(f"  路径 1 对照（{side}）：µ = {span}，出现过 {candidate_count} 个不同候选值")


def main() -> None:
    """重放指定产物目录或根目录下全部 run，并按需写出 JSON。"""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--run", type=Path, action="append", help="产物目录，可重复")
    parser.add_argument("--root", type=Path, default=DEFAULT_RUN_ROOT, help="产物根目录")
    parser.add_argument(
        "--mode",
        choices=("session", "forced", "both"),
        default="both",
        help="session 走真机门禁；forced 跳过门禁在指定相位起启动；both 两种都跑",
    )
    parser.add_argument("--phase", default="active", help="forced 模式的启动相位")
    parser.add_argument("--json", type=Path, help="把结果写入指定 JSON 文件")
    arguments = parser.parse_args()

    runs = arguments.run or sorted(
        path
        for path in arguments.root.iterdir()
        if path.is_dir() and find_tactile(path) is not None
    )
    if not runs:
        raise SystemExit(f"未在 {arguments.root} 找到可用产物目录")

    payload: list[dict[str, Any]] = []
    for run_directory in runs:
        outcomes = []
        if arguments.mode in ("session", "both"):
            outcomes.append(replay(run_directory))
        if arguments.mode in ("forced", "both"):
            # forced 需要相位锚点；独立记录器没有 trace.csv，此时只能走 session。
            if phase_window(run_directory, arguments.phase) is None:
                print(f"\n{run_directory.name}：无相位 {arguments.phase}，跳过 forced 模式")
            else:
                outcomes.append(replay_forced(run_directory, phase=arguments.phase))
        online = online_friction(run_directory)
        spans = phase_ranges(run_directory)
        for outcome in outcomes:
            payload.append(
                {
                    "run": outcome.run,
                    "mode": outcome.mode,
                    "snapshots": outcome.snapshots,
                    "started": outcome.started,
                    "start_time_s": outcome.start_time_s,
                    "start_masks": [list(side) for side in outcome.start_masks],
                    "stop_reason": outcome.stop_reason,
                    "stop_time_s": outcome.stop_time_s,
                    "estimates": list(outcome.estimates),
                    "online": {side: list(values) for side, values in online.items()},
                }
            )
            report(outcome, online, spans)

    if arguments.json:
        arguments.json.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"\n已写入 {arguments.json}")


if __name__ == "__main__":
    main()
