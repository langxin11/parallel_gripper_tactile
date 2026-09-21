"""刚度预载和导纳调度的无硬件契约及理想局部接触模型验证。"""

from dataclasses import replace
import math

import pytest

from dm_grasp_core import (
    ContactStiffnessConfig,
    ContactStiffnessEstimator,
    CrankSliderKinematics,
    SecondOrderAdmittance,
    StiffnessSnapshot,
)
from dm_grasp_core.grasp.stiffness_adaptation import (
    StiffnessAdmittance,
    StiffnessAdmittanceConfig,
    StiffnessPreload,
    StiffnessPreloadConfig,
    fresh_stiffness,
)


def _snapshot(k: float, now_s: float, sample_id: int) -> StiffnessSnapshot:
    """构造一次具有独立编号的有效估计。"""
    return StiffnessSnapshot(k, True, True, now_s, sample_id, "updated")


def _preload(**kwargs) -> StiffnessPreload:
    """从刚接触的低力开始，保持探测和斜坡边界可复现。"""
    return StiffnessPreload(
        StiffnessPreloadConfig(**kwargs),
        now_s=0.0,
        closure_m=0.01,
        force_n=0.2,
        rate_n_s=1.0,
    )


@pytest.mark.parametrize("k,goal", [(100.0, 1.0), (500.0, 1.0), (2000.0, 4.0), (9000.0, 4.0)])
def test_preload_locks_bounded_force_from_independent_estimates(k: float, goal: float) -> None:
    """等效压入量产生预期底力，锁定后不随新刚度追逐目标。"""
    preload = _preload()
    for i, now in enumerate((0.01, 0.07, 0.14)):
        locked = preload.update(_snapshot(k, now, i), now_s=now, closure_m=0.0101)
        assert locked is (i == 2)
    assert preload.goal_n == pytest.approx(goal)
    assert preload.accepted_stiffness == pytest.approx(k)
    assert not preload.update(_snapshot(10000.0, 0.2, 10), now_s=0.2, closure_m=0.0102)
    assert preload.goal_n == pytest.approx(goal)


@pytest.mark.parametrize("mode", ["invalid", "stale", "repeated", "not_updated"])
def test_preload_does_not_accept_untrustworthy_updates(mode: str) -> None:
    """旧值、初值和重复观测不能组成可信辨识窗口。"""
    preload = _preload()
    for i, now in enumerate((0.01, 0.07, 0.14, 0.2)):
        snap = _snapshot(2000.0, now, i)
        if mode == "invalid":
            snap = replace(snap, valid=False)
        elif mode == "stale":
            snap = replace(snap, last_update_time_s=now - 1.0)
        elif mode == "repeated":
            snap = replace(snap, sample_id=0)
        else:
            snap = replace(snap, updated=False)
        assert not preload.update(snap, now_s=now, closure_m=0.01)
    assert preload.accepted_stiffness is None
    assert preload.stage == "identifying"


def test_preload_inconsistent_or_invalid_sample_restarts_confirmation() -> None:
    """不一致样本与无效间隔打断连续确认。"""
    preload = _preload()
    for i, (now, k) in enumerate(((0.01, 500.0), (0.08, 500.0), (0.15, 2000.0))):
        assert not preload.update(_snapshot(k, now, i), now_s=now, closure_m=0.01)
    assert not preload.update(None, now_s=0.16, closure_m=0.01)
    assert not preload.update(_snapshot(2000.0, 0.3, 4), now_s=0.3, closure_m=0.01)


def test_preload_timeout_uses_explicit_fallback() -> None:
    """无估计时按预算回退，记录中不伪造刚度。"""
    preload = _preload()
    assert not preload.update(None, now_s=2.9, closure_m=0.01)
    assert preload.update(None, now_s=3.0, closure_m=0.01)
    assert preload.goal_n == 1.0
    assert preload.accepted_stiffness is None
    assert preload.reason == "fallback_insufficient_estimate"
    assert preload.ready(3.0)


@pytest.mark.parametrize("locked", [False, True])
def test_preload_closure_budget_applies_before_and_after_lock(locked: bool) -> None:
    """锁定目标后仍不能绕过实测闭合预算。"""
    preload = _preload()
    if locked:
        preload.update(None, now_s=3.0, closure_m=0.01)
    with pytest.raises(RuntimeError, match="闭合"):
        preload.update(None, now_s=3.1, closure_m=0.0131)


def test_preload_ramp_is_continuous_and_peak_rate_limited() -> None:
    """重新设定目标保持力连续，五次斜坡满足峰值速率约束。"""
    preload = _preload()
    for i, now in enumerate((0.01, 0.07)):
        preload.update(_snapshot(2000.0, now, i), now_s=now, closure_m=0.01)
    before = preload.sample(0.14)[0]
    preload.update(_snapshot(2000.0, 0.14, 2), now_s=0.14, closure_m=0.01)
    assert preload.sample(0.14) == pytest.approx((before, 0.0, 0.0))
    assert not preload.ready(0.14)
    samples = [preload.sample(0.14 + preload.duration_s * i / 100) for i in range(101)]
    assert all(before <= force <= 4.0 + 1e-12 for force, _, _ in samples)
    assert max(abs(rate) for _, rate, _ in samples) <= 1.0 + 1e-12
    assert samples[-1] == pytest.approx((4.0, 0.0, 0.0))
    assert preload.ready(0.14 + preload.duration_s + 1e-10)


@pytest.mark.parametrize("k", [float("nan"), float("inf"), -1.0, 0.0])
def test_nonphysical_stiffness_is_not_fresh(k: float) -> None:
    """有限正刚度才允许进入调度。"""
    assert not fresh_stiffness(_snapshot(k, 1.0, 1), 1.0, 0.2)


def test_future_or_missing_timestamp_is_not_fresh() -> None:
    """时间戳缺失或来自未来时不能假定新鲜。"""
    assert not fresh_stiffness(_snapshot(500.0, 2.0, 1), 1.0, 0.2)
    assert not fresh_stiffness(replace(_snapshot(500.0, 1.0, 1), last_update_time_s=None), 1.0, 0.2)


def test_admittance_rate_limit_preserves_motion_state_and_fixed_spring() -> None:
    """参数逐步改变，接触参考、运动状态和固定虚拟刚度不重置。"""
    cfg = StiffnessAdmittanceConfig()
    scheduler = StiffnessAdmittance(cfg)
    admittance = SecondOrderAdmittance(1.0, 20.0, 10.0, 0.001, 0.002)
    factor = math.exp(cfg.relative_rate_per_s * 0.004)
    for i in range(100):
        old_mass, old_damping = admittance.mass_kg, admittance.damping_ns_m
        scheduler.update(admittance, _snapshot(2000.0, i * 0.004, i), now_s=i * 0.004, dt_s=0.004)
        assert 1 / factor - 1e-12 <= admittance.mass_kg / old_mass <= factor + 1e-12
        assert 1 / factor - 1e-12 <= admittance.damping_ns_m / old_damping <= factor + 1e-12
        assert (admittance.displacement_m, admittance.velocity_m_s) == (0.001, 0.002)
        assert admittance.stiffness_n_m == 10.0


@pytest.mark.parametrize("k", [500.0, 2000.0])
def test_preload_stiffness_seeds_requested_outer_dynamics_with_rest_handoff(k: float) -> None:
    """active 边界匹配锁定刚度，保持位置连续并清除预载残余动能。"""
    cfg = StiffnessAdmittanceConfig()
    scheduler = StiffnessAdmittance(cfg)
    admittance = SecondOrderAdmittance(0.2, 15.0, 1.0, 0.001, 0.002)

    scheduler.seed(admittance, stiffness_n_per_m=k, dt_s=0.004)

    expected_mass = (admittance.stiffness_n_m + k) / cfg.bandwidth_rad_s**2
    assert admittance.mass_kg == pytest.approx(expected_mass)
    assert admittance.damping_ns_m == pytest.approx(
        2 * cfg.damping_ratio * cfg.bandwidth_rad_s * expected_mass
    )
    assert (admittance.displacement_m, admittance.velocity_m_s) == (0.001, 0.0)
    assert admittance.acceleration_m_s2 == 0.0
    assert scheduler.filtered_k == k
    assert scheduler.reason == "seeded_from_preload"


@pytest.mark.parametrize(
    "invalid", [None, replace(_snapshot(500.0, 0.0, 1), valid=False), _snapshot(500.0, 0.0, 1)]
)
def test_admittance_freezes_on_invalid_or_stale_estimate(invalid: StiffnessSnapshot | None) -> None:
    """估计失效冻结最近参数，不能因初值改变控制。"""
    scheduler = StiffnessAdmittance(StiffnessAdmittanceConfig())
    admittance = SecondOrderAdmittance(5.0, 100.0, 0.0)
    scheduler.update(admittance, _snapshot(500.0, 0.0, 0), now_s=0.0, dt_s=0.004)
    previous = (admittance.mass_kg, admittance.damping_ns_m, scheduler.filtered_k)
    scheduler.update(admittance, invalid, now_s=1.0, dt_s=0.004)
    assert (admittance.mass_kg, admittance.damping_ns_m, scheduler.filtered_k) == previous
    assert scheduler.reason == "frozen_stale_estimate"


@pytest.mark.parametrize("k", [500.0, 2000.0])
def test_ideal_elastic_contact_converges_without_large_overshoot(k: float) -> None:
    """仅验证无延迟理想内环的局部弹性模型，不作为真机稳定证明。"""
    scheduler = StiffnessAdmittance(StiffnessAdmittanceConfig())
    admittance = SecondOrderAdmittance(5.0, 100.0, 0.0)
    target = k * 0.002
    forces = []
    for i in range(2500):
        now = i * 0.004
        scheduler.update(admittance, _snapshot(k, now, i), now_s=now, dt_s=0.004)
        admittance.step(target - k * admittance.displacement_m, 0.004)
        forces.append(k * admittance.displacement_m)
    assert all(math.isfinite(force) for force in forces)
    assert forces[-1] == pytest.approx(target, rel=0.01)
    assert min(forces) >= 0.0
    assert max(forces) <= 1.2 * target


@pytest.mark.parametrize("before,after", [(500.0, 2000.0), (2000.0, 500.0)])
def test_ideal_contact_stiffness_jump_remains_bounded_and_recovers(
    before: float, after: float
) -> None:
    """刚度突变的瞬时力跳变不可消除，验证其后的有限响应与恢复。"""
    scheduler = StiffnessAdmittance(StiffnessAdmittanceConfig())
    admittance = SecondOrderAdmittance(before / 100.0, before / 5.0, 0.0, 1.0 / before)
    target = 1.0
    forces = []
    for i in range(3000):
        k = before if i < 250 else after
        now = i * 0.004
        scheduler.update(admittance, _snapshot(k, now, i), now_s=now, dt_s=0.004)
        admittance.step(target - k * admittance.displacement_m, 0.004)
        forces.append(k * admittance.displacement_m)
    assert all(math.isfinite(force) for force in forces)
    assert max(forces) <= 1.2 * max(target, target * after / before)
    assert min(forces) >= 0.0
    assert forces[-1] == pytest.approx(target, rel=0.01)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"indentation_m": 0.0},
        {"timeout_s": float("nan")},
        {"min_updates": True},
        {"min_updates": 2.5},
        {"min_updates": 1},
        {"max_force_n": 0.5},
        {"probe_force_n": 10.0},
        {"consistency": 1.0},
        {"stable_s": 3.0},
    ],
)
def test_preload_rejects_invalid_configuration(kwargs: dict) -> None:
    """非法辨识边界在控制运行前报错。"""
    with pytest.raises(ValueError):
        StiffnessPreloadConfig(**kwargs)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"bandwidth_rad_s": 0.0},
        {"damping_ratio": -1.0},
        {"filter_tau_s": float("inf")},
        {"relative_rate_per_s": True},
        {"min_mass_kg": 400.0},
    ],
)
def test_admittance_rejects_invalid_configuration(kwargs: dict) -> None:
    """非法调度边界在控制运行前报错。"""
    with pytest.raises(ValueError):
        StiffnessAdmittanceConfig(**kwargs)


@pytest.mark.parametrize("dt", [0.0, -1.0, float("nan"), float("inf")])
def test_admittance_rejects_invalid_time_step(dt: float) -> None:
    """非法积分周期不能进入指数限速。"""
    with pytest.raises(ValueError):
        StiffnessAdmittance(StiffnessAdmittanceConfig()).update(
            SecondOrderAdmittance(5.0, 100.0, 0.0), _snapshot(500.0, 0.0, 0), now_s=0.0, dt_s=dt
        )


def test_admittance_rejects_local_discrete_instability() -> None:
    """极端采样周期不能越过局部离散稳定边界。"""
    with pytest.raises(RuntimeError, match="离散稳定"):
        StiffnessAdmittance(StiffnessAdmittanceConfig()).update(
            SecondOrderAdmittance(5.0, 100.0, 0.0), _snapshot(2000.0, 0.0, 0), now_s=0.0, dt_s=1.0
        )


@pytest.mark.parametrize("rate", [0.0, -1.0, float("nan"), float("inf")])
def test_preload_rejects_invalid_ramp_rate(rate: float) -> None:
    """斜坡速率无效时不能创建预载控制段。"""
    with pytest.raises(ValueError):
        StiffnessPreload(
            StiffnessPreloadConfig(), now_s=0.0, closure_m=0.0, force_n=0.2, rate_n_s=rate
        )


def test_admittance_rejects_zero_damping_instead_of_staying_undamped() -> None:
    """乘法限速无法从零阻尼启动，因此显式拒绝该初态。"""
    with pytest.raises(ValueError, match="正阻尼"):
        StiffnessAdmittance(StiffnessAdmittanceConfig()).update(
            SecondOrderAdmittance(5.0, 0.0, 0.0), _snapshot(500.0, 0.0, 0), now_s=0.0, dt_s=0.004
        )


def test_preload_repeated_last_sample_cannot_supply_confirmation_time() -> None:
    """快速收集三个样本后重复末包，不能伪造稳定观测时长。"""
    preload = _preload()
    for i, now in enumerate((0.0, 0.01, 0.02)):
        assert not preload.update(_snapshot(2000.0, now, i), now_s=now, closure_m=0.01)
    assert not preload.update(_snapshot(2000.0, 0.02, 2), now_s=0.12, closure_m=0.01)
    assert preload.update(_snapshot(2000.0, 0.13, 3), now_s=0.13, closure_m=0.01)
    assert preload.accepted_stiffness == 2000.0


def test_frozen_admittance_still_checks_lengthened_time_step() -> None:
    """冻结参数不能绕过周期突然变长后的数值保护。"""
    scheduler = StiffnessAdmittance(StiffnessAdmittanceConfig())
    admittance = SecondOrderAdmittance(5.0, 100.0, 0.0)
    scheduler.update(admittance, _snapshot(500.0, 0.0, 0), now_s=0.0, dt_s=0.004)
    old_parameters = (admittance.mass_kg, admittance.damping_ns_m)
    with pytest.raises(RuntimeError, match="离散稳定"):
        scheduler.update(admittance, None, now_s=1.0, dt_s=1.0)
    assert (admittance.mass_kg, admittance.damping_ns_m) == old_parameters


@pytest.mark.parametrize("k,target", [(500.0, 1.0), (2000.0, 4.0)])
def test_actual_window_estimator_drives_preload_from_linear_contact(
    k: float, target: float
) -> None:
    """真实窗口拟合器消费线性力位移数据，经快照确认后形成接触目标。"""
    kinematics = CrankSliderKinematics(math.pi / 4, 0.03, 0.04, 0.021213203435596423)
    config = ContactStiffnessConfig(
        enabled=True,
        initial_n_per_m=1000.0,
        min_n_per_m=100.0,
        max_n_per_m=10000.0,
        filter_alpha=1.0,
        min_delta_closure_m=1e-5,
        min_delta_force_n=0.01,
        method="window_linear",
        window_size=25,
        min_samples=8,
    )
    estimator = ContactStiffnessEstimator(config, kinematics)
    preload = _preload()
    accepted_snapshots = []
    for i in range(100):
        now = i * 0.01
        requested_closure = 0.01 + i * 1e-5
        position = kinematics.position_for_closure(requested_closure, 0.0, math.pi / 2)
        closure = kinematics.closure(position)
        # 非零截距验证估计器拟合斜率，而非将现有接触力误当成刚度。
        force = 0.2 + k * (closure - 0.01)
        estimator.update(position_rad=position, normal_force_n=force)
        snapshot = estimator.snapshot(time_s=now, sample_id=i)
        if snapshot.updated:
            accepted_snapshots.append(snapshot)
        if preload.update(snapshot, now_s=now, closure_m=closure):
            break
    assert len(accepted_snapshots) >= 3
    assert estimator.estimate_n_per_m == pytest.approx(k, rel=1e-8)
    assert preload.stage == "establishing"
    assert preload.accepted_stiffness == pytest.approx(k, rel=1e-8)
    assert preload.goal_n == pytest.approx(target, rel=1e-8)


def test_preload_sparse_samples_do_not_reuse_expired_confirmation_entries() -> None:
    """每包自身新鲜但历史确认项过期时，不能拼接成可信窗口。"""
    preload = _preload()
    for i, now in enumerate((0.0, 0.19, 0.38)):
        assert not preload.update(_snapshot(2000.0, now, i), now_s=now, closure_m=0.01)
    assert preload.accepted_stiffness is None
    assert preload.stage == "identifying"


def test_preload_confirmation_uses_actual_estimate_times_not_delivery_times() -> None:
    """延迟投递不能把短时间内的估计包装成长时间稳定观测。"""
    preload = _preload()
    for i, (sample_time, delivery_time) in enumerate(((0.0, 0.01), (0.01, 0.07), (0.02, 0.14))):
        assert not preload.update(
            _snapshot(2000.0, sample_time, i), now_s=delivery_time, closure_m=0.01
        )
    assert preload.accepted_stiffness is None


@pytest.mark.parametrize("stable_s", [0.2, 0.3])
def test_preload_rejects_confirmation_span_not_less_than_freshness_budget(stable_s: float) -> None:
    """确认时间必须短于历史有效期，否则配置无法形成可信窗口。"""
    with pytest.raises(ValueError):
        StiffnessPreloadConfig(stable_s=stable_s, max_estimate_age_s=0.2)
