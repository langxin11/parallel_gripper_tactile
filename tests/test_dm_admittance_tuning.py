"""验证 DMgripper 导纳 Ramp 调参入口的配置和确定性调度。"""

from __future__ import annotations

from concurrent.futures import Future
import importlib.util
from pathlib import Path

import pytest

from parallel_gripper_tactile.studies.dm_admittance_tuning import (
    DMAdmittanceCandidate,
    DMAdmittanceTuningConfig,
    load_dm_admittance_tuning_config,
)
from parallel_gripper_tactile.studies.force_tracking_ablation import SeedSweep, StudyConfigError


ROOT = Path(__file__).resolve().parents[1]


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


def _protocol_module() -> object:
    """加载仓库内的导纳调参入口脚本。"""
    path = ROOT / "scripts/experiments/dm_admittance_tuning.py"
    spec = importlib.util.spec_from_file_location("dm_admittance_tuning", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
        "profile: profile.yaml\n"
        "task: ramp.yaml\n"
        "materials: [medium, hard]\n"
        "seeds: {start: 3, count: 2}\n"
        "max_workers: 2\n"
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

    assert config.profile == tmp_path / "profile.yaml"
    assert config.task == tmp_path / "ramp.yaml"
    assert len(config.conditions()) == 8
    assert config.conditions()[0][0].identifier == (
        "m0.02-b0.2-k1-fc5-v0.05-av0.05-cs0-ct0.05-aff0.5"
    )
    assert config.conditions()[-1][1:] == ("hard", 4)


def test_repository_config_scans_contact_transition_in_parallel() -> None:
    """仓库调参入口并行扫描接近和接触切换参数。"""
    config = load_dm_admittance_tuning_config(ROOT / "configs/studies/dm_admittance_tuning.yaml")

    assert config.max_workers == 4
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
        "profile: profile.yaml\n"
        "task: ramp.yaml\n"
        "materials: [medium]\n"
        "candidates:\n"
        f"  - {candidate}\n",
        encoding="utf-8",
    )

    with pytest.raises(StudyConfigError, match=message):
        load_dm_admittance_tuning_config(config_path)


def test_config_rejects_duplicate_candidates_and_nonpositive_max_workers(tmp_path: Path) -> None:
    """候选必须唯一，max_workers 必须为正整数。"""
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text(
        "profile: profile.yaml\n"
        "task: ramp.yaml\n"
        "materials: [medium]\n"
        "max_workers: 0\n"
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

    assert "max_workers" in str(error.value)
    assert "重复" in str(error.value)


def test_trace_diagnostics_uses_physical_force_and_keeps_initial_peak(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """峰值使用步进后触觉侧力，并且不受初始忽略窗口遮蔽。"""
    protocol = _protocol_module()
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

    diagnostics = protocol._trace_diagnostics(tmp_path, ignore_initial_s=0.2)  # type: ignore[attr-defined]

    assert diagnostics["force_tracking_ratio"] == 1.0
    assert diagnostics["raw_rmse_n"] == pytest.approx(0.0)
    assert diagnostics["raw_peak_abs_error_n"] == pytest.approx(1.0)


def test_parallel_dispatch_preserves_condition_order_and_ranking_is_deterministic(
    tmp_path: Path,
) -> None:
    """并行完成顺序不会改变配置顺序或候选排名。"""
    protocol = _protocol_module()
    first = _candidate(0.02, 0.2, 1.0)
    second = _candidate(0.02, 1.2, 20.0)
    incomplete = _candidate(0.04, 1.6, 20.0)
    config = DMAdmittanceTuningConfig(
        profile=tmp_path / "profile.yaml",
        task=tmp_path / "ramp.yaml",
        materials=("medium",),
        seeds=SeedSweep(start=0, count=1),
        candidates=(first, second),
        max_workers=2,
    )
    conditions = tuple(
        protocol.RunCondition(  # type: ignore[attr-defined]
            candidate=candidate,
            profile_path=tmp_path / f"{candidate.identifier}.yaml",
            task_path=config.task,
            output_root=tmp_path / "runs" / candidate.identifier,
            study_directory=tmp_path,
            object_material=material,
            sensor_noise_seed=seed,
        )
        for candidate, material, seed in config.conditions()
    )
    captured_workers: list[int] = []

    class ImmediateExecutor:
        """以可控 Future 模拟进程池，不运行 MuJoCo。"""

        def __init__(self, *, max_workers: int) -> None:
            captured_workers.append(max_workers)

        def __enter__(self) -> "ImmediateExecutor":
            return self

        def __exit__(self, *_: object) -> None:
            return None

        def submit(self, worker: object, condition: object) -> Future[dict[str, object]]:
            future: Future[dict[str, object]] = Future()
            future.set_result(worker(condition))  # type: ignore[operator]
            return future

    def fake_worker(condition: object) -> dict[str, object]:
        return {"candidate_id": condition.candidate.identifier}  # type: ignore[attr-defined]

    rows = protocol.execute_conditions(  # type: ignore[attr-defined]
        conditions,
        max_workers=config.max_workers,
        executor_factory=ImmediateExecutor,
        worker=fake_worker,
    )

    ranking = protocol.rank_candidates(  # type: ignore[attr-defined]
        [
            _aggregate_row(first, stable_runs=2, complete_runs=2, peak=0.3, rmse=0.2),
            _aggregate_row(second, stable_runs=2, complete_runs=2, peak=0.2, rmse=0.3),
            _aggregate_row(incomplete, stable_runs=2, complete_runs=1, peak=0.1, rmse=0.1),
        ]
    )

    assert captured_workers == [2]
    assert [row["candidate_id"] for row in rows] == [first.identifier, second.identifier]
    assert [row["candidate_id"] for row in ranking] == [
        second.identifier,
        first.identifier,
        incomplete.identifier,
    ]
