"""验证仿真时间控制定时器与物理步进的解耦行为。"""

import pytest

from parallel_gripper_tactile import RealtimePacer, SimulationTimer


def test_simulation_timer_ticks_at_its_own_period() -> None:
    """物理子步更细时，控制 tick 仍只按控制周期出现。"""
    timer = SimulationTimer(0.002)
    ticks = [timer.pop_due(step * 0.0001) for step in range(41)]

    assert ticks[0] == pytest.approx(0.002)
    assert ticks[20] == pytest.approx(0.002)
    assert ticks[40] == pytest.approx(0.002)
    assert sum(tick is not None for tick in ticks) == 3


def test_simulation_timer_reports_actual_elapsed_time_on_unaligned_tick() -> None:
    """非整除的控制周期在后一物理步边界触发并报告实际间隔。"""
    timer = SimulationTimer(0.0015)

    assert timer.pop_due(0.0) == pytest.approx(0.0015)
    assert timer.pop_due(0.001) is None
    assert timer.pop_due(0.002) == pytest.approx(0.002)


def test_simulation_timer_rejects_invalid_period_and_reversed_time() -> None:
    """非法周期与倒退的仿真时间必须显式失败。"""
    with pytest.raises(ValueError, match="positive"):
        SimulationTimer(0.0)

    timer = SimulationTimer(0.002)
    timer.pop_due(0.0)
    with pytest.raises(ValueError, match="backwards"):
        timer.pop_due(-0.001)


def test_realtime_pacer_maps_wall_time_to_simulation_time() -> None:
    """实时倍率只改变目标仿真时间，不改变物理积分步长。"""
    pacer = RealtimePacer(2.0, simulation_start_s=3.0, wall_start_s=10.0)

    assert pacer.target_simulation_time(10.25) == pytest.approx(3.5)

    with pytest.raises(ValueError, match="positive"):
        RealtimePacer(0.0)
