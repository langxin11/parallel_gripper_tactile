"""通用生命周期转换表的单元测试。"""

from __future__ import annotations

import pytest

from dmgripper_experiments.lifecycle import TERMINAL_PHASES, Lifecycle, LifecyclePhase


def test_full_transition_table_happy_path():
    """正常路径按序走完全部非终态阶段。"""
    lifecycle = Lifecycle()
    assert lifecycle.phase is LifecyclePhase.PREPARING
    assert not lifecycle.is_terminal
    lifecycle.mark_ready(0.0)
    lifecycle.start(1.0)
    lifecycle.confirm_contact(2.0)
    assert lifecycle.contact_segment == 1
    lifecycle.finish_contact_transition(2.2)
    lifecycle.activate(4.0)
    lifecycle.finish_task(14.0)
    lifecycle.begin_return(15.0, "release")
    lifecycle.complete_return(16.0)
    assert lifecycle.phase is LifecyclePhase.COMPLETED
    assert lifecycle.is_terminal
    assert lifecycle.elapsed_s(16.0) == 0.0


def test_contact_segment_increments_per_confirmed_contact():
    """接触段编号只在确认新的双侧接触时递增。"""
    lifecycle = Lifecycle()
    lifecycle.mark_ready(0.0)
    lifecycle.start(0.1)
    lifecycle.confirm_contact(1.0)
    lifecycle.finish_contact_transition(1.1)
    lifecycle.reenter_approach(2.0, "失接触")
    lifecycle.confirm_contact(3.0)
    assert lifecycle.contact_segment == 2


def test_cancel_before_enable_is_terminal_without_motion():
    """使能前取消进入 cancelled 终态。"""
    lifecycle = Lifecycle()
    lifecycle.mark_ready(0.0)
    lifecycle.cancel_before_enable(0.5)
    assert lifecycle.phase is LifecyclePhase.CANCELLED
    assert lifecycle.phase in TERMINAL_PHASES


def test_fault_keeps_reason_from_any_nonterminal_phase():
    """任意非终态都可以进入 fault 并保留原因。"""
    lifecycle = Lifecycle()
    lifecycle.mark_ready(0.0)
    lifecycle.start(0.1)
    lifecycle.fault(0.2, "使能确认丢失")
    assert lifecycle.phase is LifecyclePhase.FAULT
    assert lifecycle.fault_reason == "使能确认丢失"
    assert lifecycle.is_terminal


@pytest.mark.parametrize(
    ("call", "kwargs"),
    [
        ("start", {}),
        ("cancel_before_enable", {}),
        ("confirm_contact", {}),
        ("finish_contact_transition", {}),
        ("activate", {}),
        ("finish_task", {}),
        ("complete_return", {}),
    ],
)
def test_transitions_require_their_unique_source_phase(call: str, kwargs: dict):
    """每个显式转换只接受唯一来源阶段，非法来源立即抛错。"""
    lifecycle = Lifecycle()
    with pytest.raises(RuntimeError, match="非法"):
        getattr(lifecycle, call)(0.0, **kwargs)


def test_begin_return_accepts_active_and_holding_only():
    """release 只在 active／holding 有效。"""
    lifecycle = Lifecycle()
    lifecycle.mark_ready(0.0)
    lifecycle.start(0.1)
    with pytest.raises(RuntimeError, match="回位"):
        lifecycle.begin_return(0.2, "release")


def test_reenter_approach_requires_tracking_phase():
    """重接近只允许从跟踪阶段发起。"""
    lifecycle = Lifecycle()
    lifecycle.mark_ready(0.0)
    lifecycle.start(0.1)
    with pytest.raises(RuntimeError, match="重新接近"):
        lifecycle.reenter_approach(0.2, "失接触")


def test_terminal_phase_cannot_fault_again():
    """终态不能再次转入故障。"""
    lifecycle = Lifecycle()
    lifecycle.mark_ready(0.0)
    lifecycle.cancel_before_enable(0.5)
    with pytest.raises(RuntimeError, match="终态"):
        lifecycle.fault(0.6, "二次故障")
