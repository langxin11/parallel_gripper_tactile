"""从外部行为验证切向扰动增力策略。"""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from parallel_gripper_tactile.tangential_disturbance import (
    DisturbancePolicyConfig,
    TactileDisturbancePolicy,
)


def _policy(**overrides: object) -> TactileDisturbancePolicy:
    """创建响应快、时间尺度明确的策略实例。"""
    defaults: dict[str, object] = {
        "initial_force_n": 1.0,
        "max_force_n": 3.0,
        "max_force_rate_n_s": 100.0,
        "update_period_s": 0.01,
        "filter_tau_s": 0.001,
        "confirm_time_s": 0.02,
        "shear_threshold_n": 0.1,
        "fixed_step_n": 0.05,
        "min_step_n": 0.05,
        "max_step_n": 0.05,
        "contact_floor_n": 0.1,
    }
    defaults.update(overrides)
    return TactileDisturbancePolicy(DisturbancePolicyConfig(**defaults))


def _update(
    policy: TactileDisturbancePolicy,
    *,
    tangent: float = 0.0,
    signed_tangent: float | None = None,
    left: float = 1.0,
    right: float = 1.0,
    dt: float = 0.01,
    enabled: bool = True,
) -> object:
    """以可读的物理输入驱动一次策略更新。"""
    return policy.update(
        left_normal_n=left,
        right_normal_n=right,
        tangential_force_n=tangent,
        signed_tangential_force_n=tangent if signed_tangent is None else signed_tangent,
        closure_m=0.0,
        dt=dt,
        enabled=enabled,
    )


def _prime(policy: TactileDisturbancePolicy) -> None:
    """建立初始稳定保持的触觉基线。"""
    _update(policy, enabled=False)


def test_stable_tactile_history_never_requests_more_force() -> None:
    """稳定双侧接触在长保持中维持初始目标。"""
    policy = _policy()
    _prime(policy)

    commands = [_update(policy) for _ in range(20)]

    assert {command.target_force_n for command in commands} == {1.0}
    assert all(not command.trigger_active for command in commands)
    assert commands[-1].increase_count == 0


def test_growing_tangential_force_causes_monotonic_increase() -> None:
    """持续增长的切向触觉证据在确认后抬升目标，且绝不反向。"""
    policy = _policy(strategy="fixed_step")
    _prime(policy)

    commands = [_update(policy, tangent=value) for value in (0.03, 0.08, 0.13, 0.18, 0.22, 0.25)]

    targets = [command.target_force_n for command in commands]
    assert max(targets) > 1.0
    assert targets == sorted(targets)
    assert commands[-1].increase_count > 0


def test_unloading_tangential_force_does_not_lower_target() -> None:
    """获得一次增力后，即使载荷卸除也保持已经达到的目标。"""
    policy = _policy(strategy="fixed_step")
    _prime(policy)
    for _ in range(5):
        increased = _update(policy, tangent=0.3)
    before_unload = increased.target_force_n

    unloaded = [_update(policy, tangent=0.0) for _ in range(8)]

    assert before_unload > 1.0
    assert all(command.target_force_n >= before_unload for command in unloaded)
    assert [command.target_force_n for command in unloaded] == sorted(
        command.target_force_n for command in unloaded
    )


def test_constant_strategy_does_not_increase_after_confirmed_evidence() -> None:
    """constant 策略保留检测诊断，但不因触发改变目标。"""
    policy = _policy(strategy="constant")
    _prime(policy)

    commands = [_update(policy, tangent=0.3) for _ in range(6)]

    assert any(command.trigger_active for command in commands)
    assert {command.target_force_n for command in commands} == {1.0}
    assert commands[-1].increase_count == 0


def test_force_ratio_with_zero_normal_change_never_triggers() -> None:
    """法向力不变时，force_ratio 不把零分母伪造成触发证据。"""
    policy = _policy(detector="force_ratio", ratio_threshold=-0.05)
    _prime(policy)

    commands = [_update(policy, tangent=0.3, signed_tangent=-0.3) for _ in range(12)]

    assert all(command.ratio is None and not command.ratio_valid for command in commands)
    assert all(not command.trigger_active for command in commands)
    assert {command.target_force_n for command in commands} == {1.0}


def test_both_sides_must_remain_in_contact_before_force_increase() -> None:
    """任一侧失去连续接触时，强触觉证据也不得引发增力。"""
    policy = _policy(strategy="fixed_step")
    _prime(policy)

    unprotected = [_update(policy, tangent=0.4, right=0.0) for _ in range(8)]
    protected = [_update(policy, tangent=0.4) for _ in range(5)]

    assert {command.target_force_n for command in unprotected} == {1.0}
    assert all(not command.trigger_active for command in unprotected)
    assert protected[-1].target_force_n > 1.0


def test_rate_limit_holds_across_control_timestep_changes() -> None:
    """改变控制 dt 时，任一次目标变化都不超过物理速率上限。"""
    policy = _policy(
        strategy="fixed_step",
        fixed_step_n=0.4,
        max_force_rate_n_s=0.5,
        confirm_time_s=0.001,
    )
    _prime(policy)

    previous = 1.0
    for dt in (0.003, 0.02, 0.007, 0.04, 0.01):
        command = _update(policy, tangent=0.8, dt=dt)
        assert command.target_force_n - previous <= 0.5 * dt + 1e-12
        assert command.target_force_rate_n_s <= 0.5 + 1e-12
        previous = command.target_force_n


def test_larger_second_disturbance_restores_response_after_unloading() -> None:
    """首段扰动达到其载荷包络后，更大的再次扰动仍可继续响应。"""
    policy = _policy(strategy="fixed_step", max_force_n=2.0)
    _prime(policy)
    for _ in range(8):
        first = _update(policy, tangent=0.2)
    for _ in range(5):
        _update(policy, tangent=0.0)
    for _ in range(8):
        second = _update(policy, tangent=0.5)

    assert first.target_force_n > 1.0
    assert second.target_force_n > first.target_force_n
    assert second.increase_count > first.increase_count


@pytest.mark.parametrize(
    "overrides",
    [
        {"max_force_n": 0.5, "initial_force_n": 1.0},
        {"min_step_n": 0.2, "max_step_n": 0.1},
        {"update_period_s": 0.0},
        {"unexpected": 1},
    ],
)
def test_policy_configuration_rejects_invalid_limits_and_fields(
    overrides: dict[str, object],
) -> None:
    """无效策略配置在构造时明确失败。"""
    with pytest.raises(ValidationError):
        DisturbancePolicyConfig(**overrides)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"dt": 0.0},
        {"dt": -0.01},
        {"tangent": -0.1},
        {"tangent": math.nan},
    ],
)
def test_policy_rejects_nonphysical_runtime_inputs(kwargs: dict[str, float]) -> None:
    """非法运行时触觉输入不得悄悄改变内部状态。"""
    policy = _policy()
    _prime(policy)

    with pytest.raises(ValueError):
        _update(policy, **kwargs)


@pytest.mark.parametrize("sign, expected", [(-1, True), (1, False)])
def test_force_ratio_preserves_sign_and_can_trigger(sign: int, expected: bool) -> None:
    """相同幅值的力增量因符号不同得到不同判据，不退化成绝对值检测。"""
    policy = _policy(detector="force_ratio", ratio_threshold=-0.1)
    _prime(policy)
    commands = [
        _update(policy, left=1.0 + 0.04 * i, tangent=0.1 * i, signed_tangent=sign * 0.1 * i)
        for i in range(1, 15)
    ]
    assert any(command.ratio_valid for command in commands)
    assert any(command.trigger_active for command in commands) is expected
    assert (commands[-1].target_force_n > 1.0) is expected
