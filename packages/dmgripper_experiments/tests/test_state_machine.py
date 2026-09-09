"""验证最小真机力跟踪状态机。"""

from dmgripper_experiments import ForceDemoConfig, ForceTrackingState, ForceTrackingStateMachine


def test_contact_and_release_both_require_stable_duration() -> None:
    """短促接触或掉力不会让状态机反复切换。"""
    machine = ForceTrackingStateMachine(ForceDemoConfig())
    machine.begin_approach(0.0, "开始")
    machine.observe_forces(0.3, 0.3, 0.01)
    machine.observe_forces(0.0, 0.0, 0.05)
    machine.observe_forces(0.3, 0.3, 0.10)
    machine.observe_forces(0.3, 0.3, 0.21)
    assert machine.state is ForceTrackingState.CONTACT_TRANSITION

    machine.finish_contact_transition(0.36)
    machine.observe_forces(0.0, 0.0, 0.40)
    machine.observe_forces(0.2, 0.2, 0.50)
    assert machine.state is ForceTrackingState.FORCE_TRACKING
    machine.observe_forces(0.0, 0.0, 0.60)
    machine.observe_forces(0.0, 0.0, 0.76)
    assert machine.state is ForceTrackingState.APPROACH


def test_imbalance_is_allowed_during_approach_but_faults_after_contact() -> None:
    """单指先接触不误停，稳定接触后的持续大力差触发故障。"""
    machine = ForceTrackingStateMachine(ForceDemoConfig())
    machine.begin_approach(0.0, "开始")
    machine.observe_forces(1.5, 0.0, 0.1)
    assert machine.state is ForceTrackingState.APPROACH
    machine.observe_forces(0.3, 0.3, 0.2)
    machine.observe_forces(0.3, 0.3, 0.31)
    machine.finish_contact_transition(0.46)
    machine.observe_forces(1.5, 0.3, 0.5)
    assert machine.state is ForceTrackingState.FAULT


def test_force_ceiling_faults_in_approach() -> None:
    """任一侧超力在接近阶段也立即触发故障。"""
    machine = ForceTrackingStateMachine(ForceDemoConfig())
    machine.begin_approach(0.0, "开始")
    machine.observe_forces(2.01, 0.0, 0.1)
    assert machine.state is ForceTrackingState.FAULT
