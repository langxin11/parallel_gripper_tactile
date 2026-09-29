"""刚度闭环的配置、生命周期、目标衔接与故障集成测试。"""

from dataclasses import replace
from pathlib import Path
import csv
import json

import pytest
from dm_grasp_core import StiffnessSnapshot

from dmgripper_experiments.config import load_experiment_config
from dmgripper_experiments.observation import StiffnessDiagnostics
from dmgripper_experiments.targets import UnifiedAdaptiveTargetSource
from .fakes import FakeTactile, PhaseActions, run_fake_experiment


def _config():
    """加载独立的刚度预载测试配置，不依赖真机入口调参。"""
    return load_experiment_config(Path(__file__).parent / "fixtures/stiffness_preload.yaml")


class ReleaseActive(PhaseActions):
    """进入 active 后保留数帧以检查目标连续性。"""

    def __init__(self):
        """初始化释放计数。"""
        super().__init__()
        self.cycles = 0

    def __call__(self):
        """运行若干周期后请求回位。"""
        if self.phase == "active":
            self.cycles += 1
            if self.cycles >= 15:
                return "release"
        return super().__call__()


class NineContact(FakeTactile):
    """提供稳定九点接触，隔离验证目标与生命周期编排。"""

    def _snapshot(self, *, force_n, tangential_n=0.0):
        value = 4.0 if force_n else 0.0
        sample = super()._snapshot(force_n=value)
        return replace(
            sample,
            left_taxel_forces_n=((0.0, 0.0, value / 9),) * 9,
            right_taxel_forces_n=((0.0, 0.0, value / 9),) * 9,
        )


@pytest.mark.parametrize(
    "stiffness,goal", [(250.0, 0.5), (255.5, 0.511), (500.0, 1.0), (2000.0, 4.0), (None, 0.5)]
)
def test_preload_lock_handoff_and_diagnostics(tmp_path, monkeypatch, stiffness, goal):
    """目标只锁定一次，进入 active 保持底力，双估计链和 MIT 增益保留。"""

    def estimate(self, *, time_s, sample_id, **kwargs):
        return StiffnessSnapshot(
            stiffness or 3000.0,
            stiffness is not None,
            stiffness is not None,
            time_s if stiffness else None,
            sample_id if stiffness else None,
            "updated" if stiffness else "insufficient_excitation",
        )

    monkeypatch.setattr(StiffnessDiagnostics, "update", estimate)
    config = _config()
    assert config.timing.max_control_gap_s == 0.15
    assert config.reference.unified.friction.safety_factor == 1.2
    assert config.reference.unified.observer.min_normal_n == 0.08
    assert config.lifecycle.stiffness_preload.min_force_n == 0.5
    assert config.lifecycle.stiffness_preload.probe_force_n == 0.5
    assert config.lifecycle.stiffness_preload.max_closure_m == 0.003
    result, session, _, directory = run_fake_experiment(
        tmp_path, config, tactile_type=NineContact, actions_type=ReleaseActive
    )
    assert result["status"] == "completed" and session.disable_calls == 1
    with (directory / "trace.csv").open() as file:
        rows = list(csv.DictReader(file))
    events = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
    locks = [event for event in events if event["event"] == "stiffness_preload_locked"]
    seeds = [event for event in events if event["event"] == "admittance_seeded"]
    assert len(locks) == 1
    assert locks[0]["contact_force_goal_n"] == pytest.approx(goal)
    assert locks[0]["stiffness_preload_reason"] == (
        "stiffness_locked" if stiffness else "fallback_insufficient_estimate"
    )
    assert len(seeds) == (1 if stiffness else 0)
    if stiffness:
        expected_mass = (config.controller.admittance.stiffness_n_m + stiffness) / (
            config.controller.admittance.stiffness_adaptation.bandwidth_rad_s**2
        )
        assert seeds[0]["admittance_mass_kg"] == pytest.approx(expected_mass)
        assert seeds[0]["admittance_damping_ns_m"] == pytest.approx(
            2
            * config.controller.admittance.stiffness_adaptation.damping_ratio
            * config.controller.admittance.stiffness_adaptation.bandwidth_rad_s
            * expected_mass
        )
    active = [row for row in rows if row["phase"] == "active"]
    assert active
    assert all(float(row["target_force_n"]) == pytest.approx(goal) for row in active)
    assert all(row["adaptive_left_mu"] and row["stiffness_reason"] for row in active)
    assert all(
        float(row["kp"]) == config.controller.mit_kp
        and float(row["kd"]) == config.controller.mit_kd
        for row in active
    )
    assert all(row["admittance_mass_kg"] and row["admittance_damping_ns_m"] for row in active)
    assert all(
        row["admittance_displacement_m"]
        and row["admittance_velocity_m_s"]
        and row["admittance_acceleration_m_s2"]
        for row in active
    )
    preload = [row for row in rows if row["phase"] == "preload"]
    assert all(row["admittance_adaptation_reason"] == "waiting_estimate" for row in preload)
    expected_reason = "adapting" if stiffness else "waiting_estimate"
    assert all(row["admittance_adaptation_reason"] == expected_reason for row in active)
    assert max(abs(float(row["target_force_rate_n_s"])) for row in preload) <= 1.0 + 1e-9


def test_preload_independent_raw_force_ceiling_enters_fault_holding(tmp_path):
    """预载独立原始过力保护先于全程 40 N 上限。"""

    class Overforce(NineContact):
        def latest(self):
            sample = super().latest()
            return replace(sample, raw_left_fz_n=5.1) if self.phase.phase == "preload" else sample

    with pytest.raises(RuntimeError, match="独立保护"):
        run_fake_experiment(tmp_path, _config(), tactile_type=Overforce, actions_type=ReleaseActive)
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text())
    assert manifest["fault_holding_entered"]
    assert manifest["fault_resolution"] == "released"


@pytest.mark.parametrize("change", ["estimator", "authorization", "rate", "bound", "timeout"])
def test_stiffness_closed_loop_rejects_incompatible_config(change):
    """开启控制消费后不能静默失去估计、授权或预载边界。"""
    config = _config()
    with pytest.raises(ValueError):
        if change == "estimator":
            replace(config, estimation=replace(config.estimation, enabled=False))
        elif change == "authorization":
            replace(
                config,
                reference=replace(config.reference, experimental_closed_loop=False),
            )
        elif change == "rate":
            replace(config, lifecycle=replace(config.lifecycle, preload_force_rate_n_s=None))
        elif change == "bound":
            replace(
                config,
                lifecycle=replace(
                    config.lifecycle,
                    stiffness_preload=replace(
                        config.lifecycle.stiffness_preload, max_force_n=31.0, force_ceiling_n=32.0
                    ),
                ),
            )
        else:
            replace(config, lifecycle=replace(config.lifecycle, preload_timeout_s=3.0))


def test_contact_floor_is_one_shot_and_both_switches_can_be_disabled():
    """两个开关可独立关闭，锁定目标不允许被逐帧改写。"""
    config = _config()
    replace(config, lifecycle=replace(config.lifecycle, stiffness_preload=None))
    replace(
        config,
        controller=replace(
            config.controller,
            admittance=replace(config.controller.admittance, stiffness_adaptation=None),
        ),
    )
    source = UnifiedAdaptiveTargetSource(config.reference, config.unified_core_config)
    source.set_contact_floor(4.0)
    assert source.preload_target(0.0) == 4.0
    with pytest.raises(RuntimeError):
        source.set_contact_floor(2.0)
