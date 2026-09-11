"""验证 DMgripper 导纳 Ramp 调参研究的配置和串行确定性调度。"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from parallel_gripper_tactile.experiments.force_tracking import ForceTrackingResult
from parallel_gripper_tactile.research import compose_research_run
from parallel_gripper_tactile.studies.dm_admittance_tuning import (
    DMAdmittanceCandidate,
    load_dm_admittance_tuning_config,
)
from parallel_gripper_tactile.studies.force_tracking_ablation import StudyConfigError
from parallel_gripper_tactile.studies.protocols import dm_admittance_tuning as protocol


ROOT = Path(__file__).resolve().parents[1]


def _resolved_profile():
    """返回正式调参入口使用的组合后导纳 profile。"""
    return compose_research_run(
        experiment="dm_gripper/force_tracking_admittance",
        overrides=("execution=plan",),
    ).profile


def _candidate(
    mass_kg: float,
    damping_ns_m: float,
    stiffness_n_m: float,
    *,
    filter_cutoff_hz: float = 5.0,
    velocity_limit_rad_s: float = 0.05,
    approach_velocity_rad_s: float | None = None,
    contact_stable_time_s: float = 0.0,
    contact_transition_time_s: float = 0.05,
    approach_feedforward_force_n: float = 0.5,
) -> DMAdmittanceCandidate:
    """构造一组含接触切换参数的完整候选。"""
    return DMAdmittanceCandidate(
        mass_kg=mass_kg,
        damping_ns_m=damping_ns_m,
        stiffness_n_m=stiffness_n_m,
        filter_cutoff_hz=filter_cutoff_hz,
        velocity_limit_rad_s=velocity_limit_rad_s,
        approach_velocity_rad_s=(
            velocity_limit_rad_s if approach_velocity_rad_s is None else approach_velocity_rad_s
        ),
        contact_stable_time_s=contact_stable_time_s,
        contact_transition_time_s=contact_transition_time_s,
        approach_feedforward_force_n=approach_feedforward_force_n,
    )


def _candidate_payload(**overrides: float) -> dict[str, float]:
    """构造可按字段覆盖的完整 YAML 候选映射。"""
    payload = {
        "mass_kg": 0.02,
        "damping_ns_m": 0.2,
        "stiffness_n_m": 1.0,
        "filter_cutoff_hz": 5.0,
        "velocity_limit_rad_s": 0.05,
        "approach_velocity_rad_s": 0.05,
        "contact_stable_time_s": 0.0,
        "contact_transition_time_s": 0.05,
        "approach_feedforward_force_n": 0.5,
    }
    payload.update(overrides)
    return payload


def _aggregate_row(
    candidate: DMAdmittanceCandidate,
    *,
    stable_runs: int,
    complete_runs: int,
    peak: float,
    rmse: float,
) -> dict[str, object]:
    """构造一行候选聚合结果。"""
    return {
        "candidate_id": candidate.identifier,
        "mass_kg": candidate.mass_kg,
        "damping_ns_m": candidate.damping_ns_m,
        "stiffness_n_m": candidate.stiffness_n_m,
        "filter_cutoff_hz": candidate.filter_cutoff_hz,
        "velocity_limit_rad_s": candidate.velocity_limit_rad_s,
        "approach_velocity_rad_s": candidate.approach_velocity_rad_s,
        "contact_stable_time_s": candidate.contact_stable_time_s,
        "contact_transition_time_s": candidate.contact_transition_time_s,
        "approach_feedforward_force_n": candidate.approach_feedforward_force_n,
        "runs": 2,
        "stable_runs": stable_runs,
        "complete_force_tracking_runs": complete_runs,
        "force_tracking_ratio_mean": 1.0,
        "force_tracking_ratio_min": 1.0,
        "rmse_n_mean": rmse,
        "mae_n_mean": 0.1,
        "peak_abs_error_n_mean": peak,
        "final_error_n_mean": 0.05,
        "raw_rmse_n_mean": rmse,
        "raw_mae_n_mean": 0.1,
        "raw_peak_abs_error_n_mean": peak,
    }


def test_config_resolves_paths_and_expands_candidates(tmp_path: Path) -> None:
    """配置解析相对路径，并以候选、材料、seed 顺序展开条件。"""
    config_path = tmp_path / "tuning.yaml"
    config_path.write_text(
        "task: ramp.yaml\n"
        "materials: [medium, hard]\n"
        "seeds: {start: 3, count: 2}\n"
        "candidates:\n"
        "  - {mass_kg: 0.02, damping_ns_m: 0.2, stiffness_n_m: 1.0, "
        "filter_cutoff_hz: 5.0, velocity_limit_rad_s: 0.05, "
        "approach_velocity_rad_s: 0.05, contact_stable_time_s: 0.0, "
        "contact_transition_time_s: 0.05, approach_feedforward_force_n: 0.5}\n"
        "  - {mass_kg: 0.04, damping_ns_m: 1.0, stiffness_n_m: 20.0, "
        "filter_cutoff_hz: 10.0, velocity_limit_rad_s: 0.1, "
        "approach_velocity_rad_s: 0.1, contact_stable_time_s: 0.02, "
        "contact_transition_time_s: 0.2, approach_feedforward_force_n: 1.0}\n",
        encoding="utf-8",
    )

    config = load_dm_admittance_tuning_config(config_path)

    assert config.task == tmp_path / "ramp.yaml"
    assert len(config.conditions()) == 8
    assert config.conditions()[0][0].identifier == (
        "m0.02-b0.2-k1-fc5-v0.05-av0.05-cs0-ct0.05-aff0.5"
    )
    assert config.conditions()[-1][1:] == ("hard", 4)


def test_repository_config_scans_contact_transition_parameters() -> None:
    """仓库调参入口扫描接近和接触切换参数。"""
    config = load_dm_admittance_tuning_config(
        ROOT / "configs/research/dm_admittance_tuning/study.yaml"
    )

    assert len(config.candidates) == 16
    assert (
        _candidate(
            0.2,
            15.0,
            1.0,
            filter_cutoff_hz=2.0,
            velocity_limit_rad_s=0.05,
            contact_stable_time_s=0.0,
            contact_transition_time_s=0.05,
            approach_feedforward_force_n=0.5,
        )
        in config.candidates
    )
    assert (
        _candidate(
            0.2,
            15.0,
            1.0,
            filter_cutoff_hz=2.0,
            velocity_limit_rad_s=0.1,
            contact_stable_time_s=0.02,
            contact_transition_time_s=0.2,
            approach_feedforward_force_n=1.0,
        )
        in config.candidates
    )


@pytest.mark.parametrize(
    ("candidate", "message"),
    [
        (
            _candidate_payload(mass_kg=0.0),
            "mass_kg",
        ),
        (
            _candidate_payload(damping_ns_m=-0.1),
            "damping_ns_m",
        ),
        (
            _candidate_payload(stiffness_n_m=-1.0),
            "stiffness_n_m",
        ),
        (
            _candidate_payload(filter_cutoff_hz=0.0),
            "filter_cutoff_hz",
        ),
        (
            _candidate_payload(velocity_limit_rad_s=0.0),
            "velocity_limit_rad_s",
        ),
    ],
)
def test_config_rejects_invalid_admittance_parameters(
    tmp_path: Path, candidate: dict[str, float], message: str
) -> None:
    """质量必须为正，虚拟阻尼和刚度不能为负。"""
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text(
        f"task: ramp.yaml\nmaterials: [medium]\ncandidates:\n  - {candidate}\n",
        encoding="utf-8",
    )

    with pytest.raises(StudyConfigError, match=message):
        load_dm_admittance_tuning_config(config_path)


def test_config_rejects_duplicate_candidates(tmp_path: Path) -> None:
    """候选必须唯一，重复候选直接拒绝。"""
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text(
        "task: ramp.yaml\n"
        "materials: [medium]\n"
        "candidates:\n"
        "  - {mass_kg: 0.02, damping_ns_m: 0.2, stiffness_n_m: 1.0, "
        "filter_cutoff_hz: 5.0, velocity_limit_rad_s: 0.05, "
        "approach_velocity_rad_s: 0.05, contact_stable_time_s: 0.0, "
        "contact_transition_time_s: 0.05, approach_feedforward_force_n: 0.5}\n"
        "  - {mass_kg: 0.02, damping_ns_m: 0.2, stiffness_n_m: 1.0, "
        "filter_cutoff_hz: 5.0, velocity_limit_rad_s: 0.05, "
        "approach_velocity_rad_s: 0.05, contact_stable_time_s: 0.0, "
        "contact_transition_time_s: 0.05, approach_feedforward_force_n: 0.5}\n",
        encoding="utf-8",
    )

    with pytest.raises(StudyConfigError) as error:
        load_dm_admittance_tuning_config(config_path)

    assert "重复" in str(error.value)


def test_trace_diagnostics_uses_physical_force_and_keeps_initial_peak(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """峰值使用步进后触觉侧力，并且不受初始忽略窗口遮蔽。"""
    rows = [
        {
            "phase": "track_reference",
            "tracking_time_s": 0.0,
            "control_state": "force_tracking",
            "target_normal_force_n": 1.0,
            "measured_normal_force_n": 1.0,
            "left_taxel_normal_force_n": 2.0,
            "right_taxel_normal_force_n": 2.0,
        },
        {
            "phase": "track_reference",
            "tracking_time_s": 0.3,
            "control_state": "force_tracking",
            "target_normal_force_n": 1.0,
            "measured_normal_force_n": 4.0,
            "left_taxel_normal_force_n": 1.2,
            "right_taxel_normal_force_n": 0.8,
        },
    ]
    monkeypatch.setattr(protocol, "read_trace_rows", lambda _: rows)

    diagnostics = protocol._trace_diagnostics(tmp_path, ignore_initial_s=0.2)

    assert diagnostics["force_tracking_ratio"] == 1.0
    assert diagnostics["raw_rmse_n"] == pytest.approx(0.0)
    assert diagnostics["raw_peak_abs_error_n"] == pytest.approx(1.0)


def test_serial_execution_preserves_condition_order_and_ranking_is_deterministic(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """串行执行按配置顺序产出逐 run 行，候选排名只由聚合指标决定。"""
    config = load_dm_admittance_tuning_config(
        ROOT / "configs/research/dm_admittance_tuning/study.yaml"
    ).model_copy(update={"output_root": tmp_path / "studies"})
    trace_rows = [
        {
            "phase": "track_reference",
            "tracking_time_s": 0.3,
            "control_state": "force_tracking",
            "target_normal_force_n": 1.0,
            "left_taxel_normal_force_n": 1.0,
            "right_taxel_normal_force_n": 1.0,
        }
    ]

    def fake_execute(**kwargs: object):
        run_path = Path(str(kwargs["output_root"])) / f"{kwargs['run_prefix']}-synthetic"
        run_path.mkdir(parents=True)
        result = ForceTrackingResult(
            contact_time_s=0.1,
            tracking_start_time_s=0.2,
            tracking_duration_s=1.0,
            rmse_n=0.1,
            mae_n=0.1,
            peak_abs_error_n=0.2,
            mean_error_n=0.0,
            final_error_n=0.0,
            torque_saturation_ratio=0.0,
            position_saturation_ratio=0.0,
            mean_estimated_stiffness_n_per_m=1000.0,
            rise_time_s=0.1,
            overshoot_ratio=0.1,
            settling_time_s=0.2,
            simulation_stable=True,
        )
        return SimpleNamespace(path=run_path), result

    monkeypatch.setattr(protocol, "execute_force_tracking", fake_execute)
    monkeypatch.setattr(protocol, "read_trace_rows", lambda _: trace_rows)

    study_dir = protocol.run_study(
        config,
        resolved_profile=_resolved_profile(),
        study_directory=tmp_path / "study",
    )

    summary = json.loads((study_dir / "summary.json").read_text(encoding="utf-8"))
    assert [row["candidate_id"] for row in summary["runs"]] == [
        candidate.identifier for candidate, _, _ in config.conditions()
    ]
    assert not (study_dir / ".candidate_profiles").exists()

    first = _candidate(0.02, 0.2, 1.0)
    second = _candidate(0.02, 1.2, 20.0)
    incomplete = _candidate(0.04, 1.6, 20.0)
    ranking = protocol.rank_candidates(
        [
            _aggregate_row(first, stable_runs=2, complete_runs=2, peak=0.3, rmse=0.2),
            _aggregate_row(second, stable_runs=2, complete_runs=2, peak=0.2, rmse=0.3),
            _aggregate_row(incomplete, stable_runs=2, complete_runs=1, peak=0.1, rmse=0.1),
        ]
    )

    assert [row["candidate_id"] for row in ranking] == [
        second.identifier,
        first.identifier,
        incomplete.identifier,
    ]
    assert [row["rank"] for row in ranking] == [1, 2, 3]
