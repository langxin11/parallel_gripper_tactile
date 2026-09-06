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


def _enter_adjust_after_approach(controller: DiscreteForceController) -> None:
    """先执行一次接近动作，再在接触后进入 ADJUST。"""
    _stable_observation(controller, 0.0)
    approach = controller.decide(0.0)
    assert approach == controller.config.approach_step
    controller.action_applied(approach, 0.0)
    _stable_observation(controller, 1.0)
    assert controller.decide(0.1) == 0
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
    assert snapshot.hold_deadband_n == 0.5
    assert snapshot.reactivate_threshold_n == pytest.approx(1.2)


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
    release = controller.decide(1.7)
    assert controller.snapshot().state == DiscreteControlState.RELEASE
    assert release == -1


def test_positive_and_negative_actions_share_one_gain_estimate() -> None:
    """正向与反向正常动作都按有符号命令增量估计同一正增益。"""
    controller = DiscreteForceController(
        DiscreteForceControlConfig(
            target_force_n=6.0,
            max_force_n=10.0,
            stable_window_samples=3,
            delta_f_ema_alpha=0.5,
            delta_f_min_valid_samples=2,
        ),
        variant="adaptive-deadband",
    )
    _enter_adjust(controller)

    assert controller.decide(1.0) == 1
    controller.action_applied(1, 1.0)
    _stable_observation(controller, 2.0)
    assert controller.decide(1.3) == 0

    controller.set_target_force(1.0)
    assert controller.decide(1.4) == -1
    controller.action_applied(-1, 1.4)
    _stable_observation(controller, 1.4)
    assert controller.decide(1.7) == 0

    snapshot = controller.snapshot()
    assert snapshot.delta_f_tick_samples == 2
    assert snapshot.delta_f_tick_estimate_n == pytest.approx(0.8)
    assert snapshot.delta_f_tick_raw_n == pytest.approx(0.6)
    assert snapshot.model_valid
    assert snapshot.model_sample_count == 2
    assert snapshot.settled_action_id == 2
    assert snapshot.settled_delta_u == -1
    assert snapshot.settled_delta_f_n == pytest.approx(-0.6)


def test_sign_anomaly_is_exposed_as_event_but_not_used_by_model() -> None:
    """力变化方向异常的稳定动作不污染模型，但仍形成正常动作事件。"""
    controller = DiscreteForceController(
        DiscreteForceControlConfig(
            target_force_n=6.0,
            max_force_n=10.0,
            stable_window_samples=3,
            delta_f_min_valid_samples=1,
        ),
        variant="adaptive-deadband",
    )
    _enter_adjust(controller)
    assert controller.decide(1.0) == 1
    controller.action_applied(1, 1.0)
    _stable_observation(controller, 0.8)
    assert controller.decide(1.3) == 0

    snapshot = controller.snapshot()
    assert snapshot.delta_f_tick_samples == 0
    assert snapshot.delta_f_tick_estimate_n is None
    assert snapshot.settled_action_id == 1
    assert snapshot.settled_delta_u == 1
    assert snapshot.settled_delta_f_n == pytest.approx(-0.2)


def test_dynamic_candidates_can_select_multiple_negative_ticks() -> None:
    """可靠模型在当前力高于目标较多时可选择负向多 tick 动作。"""
    controller = DiscreteForceController(
        DiscreteForceControlConfig(
            target_force_n=8.0,
            max_force_n=12.0,
            stable_window_samples=3,
            delta_f_min_valid_samples=1,
            max_dynamic_step=3,
        ),
        variant="dynamic-step",
    )
    _enter_adjust_after_approach(controller)
    assert controller.decide(1.0) == 1
    controller.action_applied(1, 1.0)
    _stable_observation(controller, 2.0)
    assert controller.decide(1.3) == 0

    controller.set_target_force(2.0)
    _stable_observation(controller, 5.0)
    assert controller.decide(1.4) == -3
    snapshot = controller.snapshot()
    assert snapshot.selected_action == -3
    assert snapshot.candidate_cost_m3 == pytest.approx(0.6)
    assert snapshot.candidate_cost_hold == pytest.approx(3.0)
    assert snapshot.candidate_cost_p3 is not None


def test_dynamic_warm_up_uses_only_single_tick_actions() -> None:
    """模型可靠前即使误差很大也只允许单 tick 调整。"""
    controller = DiscreteForceController(
        DiscreteForceControlConfig(
            target_force_n=8.0,
            max_force_n=12.0,
            stable_window_samples=3,
            delta_f_min_valid_samples=2,
            max_dynamic_step=3,
        ),
        variant="dynamic-step",
    )
    _enter_adjust(controller)
    assert controller.decide(1.0) == 1
    controller.action_applied(1, 1.0)
    _stable_observation(controller, 2.0)
    assert controller.decide(1.3) == 0
    assert not controller.snapshot().model_valid
    assert controller.decide(1.4) == 1


def test_safety_release_preempts_approach_and_does_not_update_model() -> None:
    """任何普通状态下超限都立即单 tick 释放，释放稳定后不形成模型事件。"""
    controller = DiscreteForceController(
        DiscreteForceControlConfig(
            target_force_n=5.0,
            max_force_n=8.0,
            stable_window_samples=3,
            delta_f_min_valid_samples=1,
        ),
        variant="dynamic-step",
    )
    _stable_observation(controller, 0.0)
    assert controller.decide(0.0) == 3
    controller.action_applied(3, 0.0)

    _stable_observation(controller, 8.1)
    assert controller.decide(0.1) == -1
    assert controller.snapshot().state == DiscreteControlState.RELEASE
    controller.action_applied(-1, 0.1)
    _stable_observation(controller, 7.0)
    assert controller.decide(0.4) == 0

    snapshot = controller.snapshot()
    assert snapshot.delta_f_tick_samples == 0
    assert snapshot.settled_action_id == 0
    assert snapshot.settled_delta_u == 0
    assert snapshot.settled_delta_f_n is None


def test_fractional_commands_are_rejected_instead_of_truncated() -> None:
    """配置和已应用动作都不能把小数命令静默截断为整数。"""
    with pytest.raises(ValueError, match="must be integers"):
        DiscreteForceControlConfig(approach_step=1.5)  # type: ignore[arg-type]

    controller = DiscreteForceController(DiscreteForceControlConfig())
    with pytest.raises(ValueError, match="must be an integer"):
        controller.action_applied(1.5, 0.0)  # type: ignore[arg-type]
    with pytest.raises(ValueError, match=r"\[1, 3\]"):
        DiscreteForceControlConfig(max_dynamic_step=4)


def test_pending_action_can_be_cancelled_before_application() -> None:
    """安全监测可取消延迟队列中的夹紧动作，且不登记为已执行。"""
    controller = DiscreteForceController(DiscreteForceControlConfig())
    _stable_observation(controller, 0.0)
    assert controller.decide(0.0) == 3

    controller.cancel_pending_action()

    assert controller.command == 0
    assert controller.action_count == 0
    assert controller.snapshot().requested_delta == 0
