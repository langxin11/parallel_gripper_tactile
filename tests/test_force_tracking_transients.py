"""验证加载阶跃瞬态指标的纯函数评估逻辑。"""

import math

import pytest

from parallel_gripper_tactile.experiments.force_tracking import (
    ForceWaypoint,
    _evaluate_step_transients,
)


# 与 configs/force_tracking/step.yaml 同构的阶跃曲线：t_0=1.01，平台段止于 3.01。
STEP_WAYPOINTS = (
    ForceWaypoint(t_s=0.0, force_n=1.0),
    ForceWaypoint(t_s=1.0, force_n=1.0),
    ForceWaypoint(t_s=1.01, force_n=6.0),
    ForceWaypoint(t_s=3.0, force_n=6.0),
    ForceWaypoint(t_s=3.01, force_n=1.0),
    ForceWaypoint(t_s=4.0, force_n=1.0),
)


def _row(tracking_time_s: float, filtered_n: float, phase: str = "track_reference") -> dict:
    """构造评估函数所需的最小 trace 行。"""
    return {
        "phase": phase,
        "tracking_time_s": tracking_time_s,
        "filtered_normal_force_n": filtered_n,
    }


def _step_rows(forces: list[float]) -> list[dict]:
    """从 t_0=1.01 起以 0.05 s 采样合成阶跃响应，并混入窗口外的干扰行。"""
    rows = [
        _row(0.5, 1.0),
        _row(3.05, 1.0),
        _row(1.0, 1.0, phase="contact_settle"),
    ]
    rows.extend(_row(1.01 + 0.05 * index, force) for index, force in enumerate(forces))
    return rows


def test_second_order_response_reports_all_three_transients() -> None:
    """带超调后回稳的二阶风格响应给出三个已知瞬态值。"""
    forces = [
        1.0,
        3.0,
        4.6,
        5.4,
        5.8,
        6.0,
        6.45,
        6.3,
        6.15,
        6.05,
        6.0,
        6.0,
        6.0,
    ]
    rise, overshoot, settling = _evaluate_step_transients(
        _step_rows(forces),
        waypoints=STEP_WAYPOINTS,
        interpolation="hold",
        ignore_initial_s=0.2,
    )

    # 上升阈值为 1 + 0.9×5 = 5.5，首个达标采样在 t_0+0.20。
    assert rise == pytest.approx(0.20)
    # 峰值 6.45 对应超调比 (6.45−6)/5。
    assert overshoot == pytest.approx(0.09)
    # ±5% 稳定带为 ±0.25，最后一次违反在 6.3（t_0+0.35），其后一个采样进入稳态。
    assert settling == pytest.approx(0.40)


def test_never_settled_response_has_no_settling_time() -> None:
    """窗口末尾仍违反稳定带时，稳定时间为 None。"""
    forces = [1.0, 3.0, 5.6, 6.4, 6.4, 6.4, 6.4, 6.4]
    rise, overshoot, settling = _evaluate_step_transients(
        _step_rows(forces),
        waypoints=STEP_WAYPOINTS,
        interpolation="hold",
        ignore_initial_s=0.2,
    )

    assert rise == pytest.approx(0.10)
    assert overshoot == pytest.approx(0.08)
    assert settling is None


def test_no_band_violation_settles_at_first_window_sample() -> None:
    """首采样前已满足稳定带且从无违反时，稳定时间取首个采样时刻。"""
    forces = [6.0, 6.1, 6.0, 6.05, 6.0]
    rise, overshoot, settling = _evaluate_step_transients(
        _step_rows(forces),
        waypoints=STEP_WAYPOINTS,
        interpolation="hold",
        ignore_initial_s=0.2,
    )

    assert rise == pytest.approx(0.0)
    assert overshoot == pytest.approx(0.02)
    assert settling == pytest.approx(0.0)


def test_linear_interpolation_has_no_transient_metrics() -> None:
    """无阶跃语义的 linear 插值任务不产生瞬态指标。"""
    rise, overshoot, settling = _evaluate_step_transients(
        _step_rows([1.0, 2.0, 3.0, 6.0, 6.0, 6.0]),
        waypoints=STEP_WAYPOINTS,
        interpolation="linear",
        ignore_initial_s=0.2,
    )

    assert (rise, overshoot, settling) == (None, None, None)


def test_unreached_rise_level_disables_all_transients() -> None:
    """窗口内从未达到 90% 上升阈值时，三个指标均为 None。"""
    forces = [1.0, 2.0, 3.0, 4.0, 5.0, 5.4, 5.3, 5.4]
    rise, overshoot, settling = _evaluate_step_transients(
        _step_rows(forces),
        waypoints=STEP_WAYPOINTS,
        interpolation="hold",
        ignore_initial_s=0.2,
    )

    assert (rise, overshoot, settling) == (None, None, None)


def test_hold_without_qualifying_step_has_no_transient_metrics() -> None:
    """最大上升跳变不足 1 N 时视为无合格加载阶跃。"""
    waypoints = (
        ForceWaypoint(t_s=0.0, force_n=1.0),
        ForceWaypoint(t_s=1.0, force_n=1.5),
        ForceWaypoint(t_s=2.0, force_n=1.5),
    )

    rise, overshoot, settling = _evaluate_step_transients(
        [_row(0.1, 1.0), _row(1.1, 1.5)],
        waypoints=waypoints,
        interpolation="hold",
        ignore_initial_s=0.0,
    )

    assert (rise, overshoot, settling) == (None, None, None)


def test_short_plateau_window_is_rejected() -> None:
    """平台段短于 0.5 s 时不评估瞬态指标。"""
    waypoints = (
        ForceWaypoint(t_s=0.0, force_n=1.0),
        ForceWaypoint(t_s=1.0, force_n=6.0),
        ForceWaypoint(t_s=1.3, force_n=1.0),
    )

    rise, overshoot, settling = _evaluate_step_transients(
        [_row(1.0, 1.0), _row(1.1, 6.0), _row(1.2, 6.0)],
        waypoints=waypoints,
        interpolation="hold",
        ignore_initial_s=0.0,
    )

    assert (rise, overshoot, settling) == (None, None, None)


def test_ignore_initial_s_shifts_window_start() -> None:
    """窗口起点同时受 ignore_initial_s 约束，早于跳变的采样不参与统计。"""
    rows = [
        _row(0.5, 6.0),
        _row(1.0, 6.0),
        _row(1.01, 6.0),
        _row(1.06, 6.1),
        _row(1.11, 6.0),
    ]
    rise, overshoot, settling = _evaluate_step_transients(
        rows,
        waypoints=STEP_WAYPOINTS,
        interpolation="hold",
        ignore_initial_s=1.05,
    )

    # 首个纳入窗口的采样是 1.06，上升与稳定时刻均相对 t_0=1.01 计算；
    # 窗口内从无违反稳定带，稳定时间即首采样时刻。
    assert rise == pytest.approx(0.05)
    assert overshoot == pytest.approx(0.02)
    assert settling == pytest.approx(0.05)


def test_transient_metrics_are_absent_when_window_is_empty() -> None:
    """跟踪窗口为空时返回三个 None，且不产生 NaN。"""
    rise, overshoot, settling = _evaluate_step_transients(
        [_row(0.5, 1.0)],
        waypoints=STEP_WAYPOINTS,
        interpolation="hold",
        ignore_initial_s=0.2,
    )

    assert (rise, overshoot, settling) == (None, None, None)
    assert all(value is None or math.isfinite(value) for value in (rise, overshoot, settling))
