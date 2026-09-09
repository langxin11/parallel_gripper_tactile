"""验证真机力跟踪命令行的无设备路径。"""

import json

import pytest

from dmgripper_experiments.cli import run


def test_dry_run_does_not_access_hardware(capsys: pytest.CaptureFixture[str]) -> None:
    """默认模式只输出已解析计划。"""
    assert run(["--target-force", "0.4", "--duration", "3"]) == 0
    record = json.loads(capsys.readouterr().out)
    assert record["mode"] == "dry-run"
    assert record["config"]["target_force_n"] == 0.4
    assert record["config"]["tracking_duration_s"] == 3.0
    assert record["config"]["tactile_startup_timeout_s"] == 5.0
    assert record["config"]["return_settle_timeout_s"] == 2.0
    assert record["config"]["return_position_tolerance_rad"] == 0.02
    assert record["config"]["return_closure_velocity_m_s"] == 0.012
    assert record["config"]["return_mit_kp"] == 10.0
    assert record["verify_zero_force"] is True


def test_zero_check_can_be_relaxed_or_explicitly_skipped(
    capsys: pytest.CaptureFixture[str],
) -> None:
    """真机零偏参数可以通过命令行调整。"""
    assert (
        run(
            [
                "--zero-force-threshold",
                "0.25",
                "--zero-force-stable",
                "0.2",
                "--zero-force-timeout",
                "8",
                "--skip-zero-check",
                "--return-closure-velocity",
                "0.015",
            ]
        )
        == 0
    )
    record = json.loads(capsys.readouterr().out)
    assert record["config"]["zero_force_threshold_n"] == 0.25
    assert record["config"]["zero_force_stable_s"] == 0.2
    assert record["config"]["zero_force_timeout_s"] == 8.0
    assert record["config"]["return_closure_velocity_m_s"] == 0.015
    assert record["verify_zero_force"] is False


def test_invalid_target_is_rejected(capsys: pytest.CaptureFixture[str]) -> None:
    """超过默认力上限的目标不进入硬件路径。"""
    assert run(["--target-force", "3"]) == 1
    assert "目标力" in capsys.readouterr().err
