"""原厂短时会话在真实实验运行时中的离线接入验证。"""

from dataclasses import replace
import csv
import json

import pytest

from papillarray_hardware.native_slip import NativeSlipStatus
from dmgripper_experiments.config import load_experiment_config
from dmgripper_experiments.native_slip import NativeSlipConfig

from .fakes import FakeTactile, curve_config, run_fake_experiment


class NativeTactile(FakeTactile):
    """由新快照确认启停，复用既有实验替身的接触与反馈。"""

    native_slip_status = NativeSlipStatus()
    instances: list = []

    def __init__(self, *args, **kwargs):
        """记录本实例的命令序列。"""
        super().__init__(*args, **kwargs)
        self.calls = []
        self.instances.append(self)

    def request_native_slip(self, session_id, **kwargs):
        """检查运行时仅在受控接触阶段发出启动。"""
        assert self.phase.phase in {"preload", "active"}
        self.calls.append(("start", self.phase.phase, self.clock()))
        self.native_slip_status = NativeSlipStatus(
            session_id, "starting", "awaiting_active", self.clock()
        )

    def stop_native_slip(self, reason="cancelled"):
        """区分停止请求与下一包停止确认。"""
        if self.native_slip_status.phase in {"starting", "active"}:
            self.calls.append(("stop", reason, self.clock()))
            self.native_slip_status = replace(
                self.native_slip_status, phase="stopping", reason=reason
            )

    def latest(self):
        """用新包确认设备状态并提供本次会话的单点摩擦。"""
        sample = super().latest()
        status = self.native_slip_status
        if status.phase == "starting":
            status = replace(status, phase="active", reason="confirmed_active")
        elif status.phase == "stopping":
            status = replace(status, phase="stopped")
        self.native_slip_status = status
        active = status.phase == "active"
        return replace(
            sample,
            native_session_id=status.session_id,
            native_session_phase=status.phase,
            native_slip_active=(active, active),
            native_pillar_states=((3,), (3,)) if active else ((1,), (1,)),
            native_pillar_friction=((0.4,), (0.5,)) if active else ((-1.0,), (-1.0,)),
        )


def test_runtime_stable_session_records_estimates_and_stops(tmp_path):
    """稳定门槛通过后启用并冻结双侧结果，旁路不改变目标力。"""
    config = curve_config(native_slip=NativeSlipConfig(max_duration_s=1.0, stable_duration_s=0.01))
    result, _, _, directory = run_fake_experiment(tmp_path, config, tactile_type=NativeTactile)
    assert result["status"] == "completed"
    calls = NativeTactile.instances[-1].calls
    assert [call[0] for call in calls] == ["start", "stop"]
    assert calls[-1][1] == "estimates_ready"
    events = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
    estimates = [event for event in events if event["event"] == "native_slip_estimate"]
    assert [(e["side"], e["pillar_id"], e["native_mu"]) for e in estimates] == [
        ("left", 0, 0.4),
        ("right", 0, 0.5),
    ]
    with (directory / "trace.csv").open() as handle:
        rows = list(csv.DictReader(handle))
    assert any(row["native_session_phase"] == "stopped" for row in rows)
    assert {float(row["target_force_n"]) for row in rows if row["target_force_n"]} == {0.5}


def test_native_config_requires_explicit_duration_and_preserves_default(tmp_path):
    """原厂服务默认关闭，配置显式最大时长才能构造会话。"""
    assert curve_config().lifecycle.native_slip is None
    source = tmp_path / "native.yaml"
    source.write_text("lifecycle:\n  native_slip:\n    max_duration_s: 1.5\n", encoding="utf-8")
    config = load_experiment_config(source)
    assert config.lifecycle.native_slip.max_duration_s == 1.5
    source.write_text("lifecycle:\n  native_slip: {}\n", encoding="utf-8")
    with pytest.raises((TypeError, ValueError), match="max_duration_s"):
        load_experiment_config(source)
