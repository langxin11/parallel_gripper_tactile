"""验证基于单 tick 力增量的离散状态机。"""

import pytest

from parallel_gripper_tactile.discrete_force_control import (
    DiscreteControlState,
    DiscreteForceControlConfig,
    DiscreteForceController,
)


def _stable_observation(controller: DiscreteForceController, force_n: float) -> None:
    """填满控制器稳定窗口。"""
    for _ in range(controller.config.stable_window_samples):
        controller.observe(force_n)


def _enter_adjust(controller: DiscreteForceController) -> None:
    """从已接触且稳定的状态进入 ADJUST。"""
    _stable_observation(controller, 1.0)
    assert controller.decide(0.0) == 0
    assert controller.snapshot().state == DiscreteControlState.WAIT_STABLE
    assert controller.decide(0.3) == 0
    assert controller.snapshot().state == DiscreteControlState.ADJUST


def test_controller_estimates_tick_gain_only_after_settled_actions() -> None:
    """夹紧动作重新稳定后按每 tick 归一化并更新 EWMA。"""
    controller = DiscreteForceController(
        DiscreteForceControlConfig(
            target_force_n=7.0,
            max_force_n=10.0,
            stable_window_samples=4,
            delta_f_min_valid_samples=3,
        ),
        variant="adaptive-deadband",
    )
    _enter_adjust(controller)

    for index, force in enumerate((2.0, 3.0, 4.0), start=1):
        assert controller.decide(float(index)) == 1
        controller.action_applied(1, float(index))
        _stable_observation(controller, force)
        assert controller.decide(float(index) + 0.3) == 0

    snapshot = controller.snapshot()
    assert snapshot.delta_f_tick_samples == 3
    assert snapshot.delta_f_tick_estimate_n == 1.0
    assert controller.delta_f_tick_reliable
    assert snapshot.hold_deadband_n == 0.6
    assert snapshot.reactivate_threshold_n == pytest.approx(1.3)


def test_prediction_prefers_hold_to_a_worse_adjacent_tick() -> None:
    """一步预测表明相邻位置更差时保留零动作。"""
    controller = DiscreteForceController(
        DiscreteForceControlConfig(
            target_force_n=4.2,
            max_force_n=10.0,
            stable_window_samples=3,
            delta_f_min_valid_samples=1,
            tick_deadband_factor=0.1,
            prediction_tick_margin_factor=0.2,
        ),
        variant="predictive",
    )
    _enter_adjust(controller)
    assert controller.decide(1.0) == 1
    controller.action_applied(1, 1.0)
    _stable_observation(controller, 3.0)
    controller.decide(1.3)
    assert controller.snapshot().delta_f_tick_estimate_n == 2.0

    _stable_observation(controller, 3.6)
    assert controller.decide(1.4) == 0
    assert controller.snapshot().state == DiscreteControlState.HOLD


def test_dynamic_step_and_release_remain_bounded_integer_actions() -> None:
    """动态夹紧不超过上限，安全释放始终为单 tick。"""
    controller = DiscreteForceController(
        DiscreteForceControlConfig(
            target_force_n=8.0,
            max_force_n=10.0,
            stable_window_samples=3,
            delta_f_min_valid_samples=1,
            max_dynamic_step=3,
        ),
        variant="dynamic-step",
    )
    _enter_adjust(controller)
    assert controller.decide(1.0) == 1
    controller.action_applied(1, 1.0)
    _stable_observation(controller, 1.5)
    controller.decide(1.3)
    assert controller.decide(1.4) == 3

    controller.action_applied(3, 1.4)
    _stable_observation(controller, 10.1)
    controller.decide(1.7)
    release = controller.decide(1.71)
    assert controller.snapshot().state == DiscreteControlState.RELEASE
    assert release == -1
