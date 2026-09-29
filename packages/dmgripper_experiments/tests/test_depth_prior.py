"""真实 YAML、关节压缩量与统一目标来源的离线集成。"""

from dataclasses import replace
from pathlib import Path
import csv
import json

import pytest

from dmgripper_hardware import MotorFeedback, STATUS_ENABLED
from dmgripper_experiments.config import SafetyConfig, load_experiment_config
from dmgripper_experiments.observation import PairedObservation
from dmgripper_experiments.replay import replay_tactile
from dmgripper_experiments.targets import UnifiedAdaptiveTargetSource
from dmgripper_experiments.tactile import TactileSnapshot

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
REAL_CONFIGS = ("unified_adaptive", "unified_adaptive_particle")


def _depth_snapshot(index: int) -> TactileSnapshot:
    """构造一帧双侧各九触点、带均匀切向分量的快照。"""
    return TactileSnapshot(
        received_at_s=index * 0.01,
        packet_counter=index + 1,
        timestamp_us=(index + 1) * 10_000,
        left_force_n=1.8,
        right_force_n=1.8,
        raw_left_fz_n=1.8,
        raw_right_fz_n=1.8,
        raw_left_fx_n=0.0,
        raw_left_fy_n=0.05,
        raw_right_fx_n=0.0,
        raw_right_fy_n=0.0,
        left_taxel_forces_n=((0.0, 0.05 / 9, 0.2),) * 9,
        right_taxel_forces_n=((0.0, 0.05 / 9, 0.2),) * 9,
        # 逐触点位移刻意与关节闭合量不一致：先验必须只依赖后者。
        left_taxel_displacements_mm=((0.0, 0.0, 2.0),) * 9,
        right_taxel_displacements_mm=((0.0, 0.0, 1.0),) * 9,
    )


def _observed_source(name: str) -> UnifiedAdaptiveTargetSource:
    """以真实配置驱动统一目标来源完成 400 帧观测并激活。"""
    config = load_experiment_config(REPOSITORY_ROOT / f"configs/hardware/dmgripper/{name}.yaml")
    unified = config.reference.unified
    assert unified is not None
    source = UnifiedAdaptiveTargetSource(config.reference, config.unified_core_config)
    source.set_contact_closure(0.01)
    source.set_contact_closure(0.02)
    for index in range(400):
        source.observe(
            PairedObservation(
                snapshot=_depth_snapshot(index),
                feedback=MotorFeedback(0, 0, 0, STATUS_ENABLED),
                is_new_tactile=True,
                tactile_age_s=0,
                tangential_force_n=0.1,
                signed_tangential_force_n=0.05,
                closure_m=0.011,
            ),
            0.002,
        )
        if index == 0:
            source.activate()
    return source


def _expected_prior_value(name: str) -> float:
    """由配置曲线计算深度 0.001 m 处的对数先验期望值。"""
    config = load_experiment_config(REPOSITORY_ROOT / f"configs/hardware/dmgripper/{name}.yaml")
    unified = config.reference.unified
    assert unified is not None and unified.depth_friction_prior is not None
    return unified.depth_friction_prior.friction_at(0.001)


@pytest.mark.parametrize("name", REAL_CONFIGS)
def test_depth_prior_uses_joint_closure_not_taxel_displacement(name):
    """两条估计路线都用关节推算总压缩量，不受逐触点位移影响。"""
    source = _observed_source(name)
    fields = source.trace_fields()
    expected = _expected_prior_value(name)
    assert fields["adaptive_left_depth_prior_depth_m"] == pytest.approx(0.001)
    assert fields["adaptive_right_depth_prior_depth_m"] == pytest.approx(0.001)
    assert fields["adaptive_left_depth_prior_locked"]
    assert fields["adaptive_left_depth_prior_value"] == pytest.approx(expected)
    assert fields["adaptive_right_depth_prior_value"] == pytest.approx(expected)
    if name == "unified_adaptive":
        assert fields["adaptive_left_mu"] == pytest.approx(expected)
    else:
        # 粒子路线以保守分位输出：后验高于先验下限，但不高于先验点值。
        assert 0.1 < fields["adaptive_left_mu"] < expected
    assert source.failure_reason is None


def test_real_depth_config_replay_remains_diagnostic(tmp_path):
    """新增先验与粒子配置不妨碍撤销控制权限的旧记录旁路回放。"""
    config = load_experiment_config(
        REPOSITORY_ROOT / "configs/hardware/dmgripper/unified_adaptive_particle.yaml"
    )
    sample = {
        "timestamp_us": 0,
        "left_force_n": 1.8,
        "right_force_n": 1.8,
        "left_taxel_forces_n": [[0, 0, 0.2]] * 9,
        "right_taxel_forces_n": [[0, 0, 0.2]] * 9,
    }
    source, destination = tmp_path / "tactile.jsonl", tmp_path / "diagnostics.csv"
    source.write_text(json.dumps(sample), encoding="utf-8")
    assert replay_tactile(source, destination, config.unified_core_config) == 1
    with destination.open() as handle:
        row = next(csv.DictReader(handle))
    assert row["diagnostic_only"] == "True"
    assert float(row["adaptive_left_mu"]) == 0.1


@pytest.mark.parametrize("limit", [0.0, -0.02, True, float("nan"), float("inf")])
def test_invalid_compression_limit_is_rejected(limit):
    """压缩上限须为正有限量，不允许用零或布尔值伪装关闭。"""
    with pytest.raises(ValueError):
        SafetyConfig(max_contact_compression_m=limit)


def test_compression_limit_fault_holds_and_can_release(tmp_path):
    """命令越界在发送前进入故障保持，仍接受人工释放。"""
    from .fakes import run_fake_experiment
    from .test_stiffness_runtime import NineContact, ReleaseActive

    config = load_experiment_config(
        REPOSITORY_ROOT / "configs/hardware/dmgripper/unified_adaptive_particle.yaml"
    )
    config = replace(config, safety=replace(config.safety, max_contact_compression_m=1e-7))
    with pytest.raises(RuntimeError, match="最大压缩"):
        run_fake_experiment(tmp_path, config, tactile_type=NineContact, actions_type=ReleaseActive)
    manifest = json.loads(next(tmp_path.rglob("manifest.json")).read_text())
    assert manifest["fault_holding_entered"]
    assert manifest["fault_resolution"] == "released"
    events = [
        json.loads(line) for line in next(tmp_path.rglob("events.jsonl")).read_text().splitlines()
    ]
    event = next(event for event in events if event["event"] == "compression_limit")
    assert event["phase"] in {"approach", "contact_transition", "preload", "active"}
    assert event["contact_compression_limit_m"] == 1e-7


def test_prior_waits_for_bilateral_contact_while_guard_counts_unilateral_motion(tmp_path):
    """先单侧接触的运动计入行程保护，但不计入摩擦先验压入量。"""
    from .fakes import run_fake_experiment
    from .test_stiffness_runtime import NineContact, ReleaseActive

    class SingleBeforeBoth(NineContact):
        """接近阶段前三包只让左侧接触，随后双侧接触。"""

        def latest(self):
            """在真实阶段机前注入短暂单侧接触。"""
            sample = super().latest()
            if self.phase.phase == "approach":
                self.approach_frames = getattr(self, "approach_frames", 0) + 1
                if self.approach_frames <= 3:
                    return replace(
                        sample,
                        right_force_n=0.0,
                        raw_right_fz_n=0.0,
                        right_taxel_forces_n=((0.0, 0.0, 0.0),) * 9,
                    )
            return sample

    config = load_experiment_config(
        REPOSITORY_ROOT / "configs/hardware/dmgripper/unified_adaptive_particle.yaml"
    )
    result, _, _, directory = run_fake_experiment(
        tmp_path, config, tactile_type=SingleBeforeBoth, actions_type=ReleaseActive
    )
    assert result["status"] == "completed"
    with (directory / "trace.csv").open() as handle:
        rows = list(csv.DictReader(handle))
    assert any(
        row["contact_closure_m"] and not row["depth_prior_contact_closure_m"] for row in rows
    )
    paired = next(row for row in rows if row["depth_prior_contact_closure_m"])
    assert float(paired["depth_prior_contact_closure_m"]) > float(paired["contact_closure_m"])
