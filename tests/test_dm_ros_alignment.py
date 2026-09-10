"""加载真实 ROS 节点方法与仿真适配器进行离线观测回放，不创建 ROS 节点。

只有安装 ROS 及消息包、并将 DM ROS 包加入 PYTHONPATH 时运行本组集成测试。
"""

from __future__ import annotations

from dataclasses import asdict
import math
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
import yaml

pytest.importorskip("rclpy")
pytest.importorskip("dm_gripper_msgs.msg")
pytest.importorskip("dm_gripper_control")
from dm_gripper_control import force_tracking_node as ros_module
from dm_grasp_core import CrankSliderKinematics, SecondOrderAdmittance
from parallel_gripper_tactile.control import (
    ForceControlObservation,
    ForceControlReference,
    MITControlCommand,
)
from parallel_gripper_tactile.dm_admittance import DMAdmittanceController
from parallel_gripper_tactile.research import compose_research_run

ROOT = Path(__file__).resolve().parents[1]


class _ReplayMotor:
    """仅记录命令的执行器替身；观测独立来自回放轨迹，避免引入模型差异。"""

    actuator_id = 0

    def position(self, data: SimpleNamespace) -> float:
        """返回回放角度。"""
        return data.position_rad

    def velocity(self, data: SimpleNamespace) -> float:
        """返回回放角速度。"""
        return data.velocity_rad_s

    def apply(self, data: SimpleNamespace, **values: float) -> MITControlCommand:
        """返回请求的记录，不执行物理仿真或发送设备命令。"""
        return MITControlCommand(
            target_position=values["target_position"],
            target_velocity=values["target_velocity"],
            position=data.position_rad,
            velocity=data.velocity_rad_s,
            feedforward_torque=values["feedforward_torque"],
            torque=0.0,
        )


class _ROSReplay:
    """只绑定实际 ROS 控制方法；消息发布保存到内存，不创建通信实体。"""

    _control_tick = ros_module.DmGripperForceTrackingNode._control_tick
    _command_config = ros_module.DmGripperForceTrackingNode._command_config
    _to_ros_command = staticmethod(ros_module.DmGripperForceTrackingNode._to_ros_command)
    _contact_threshold_n = ros_module.DmGripperForceTrackingNode._contact_threshold_n
    _has_confirmed_dual_contact = ros_module.DmGripperForceTrackingNode._has_confirmed_dual_contact

    def __init__(self, parameters: dict[str, object]) -> None:
        """初始化确定性跟踪状态，省略已另行测试的设备使能与安全输入校验。"""
        self.parameters = parameters
        self._lock = threading.Lock()
        self._tracking_state = ros_module.TrackingState.FORCE_TRACKING
        self._reference_position_rad = 0.5
        self._arming_started_s = None
        self._last_control_s = 0.0
        self._admittance = SecondOrderAdmittance(
            parameters["admittance_mass_kg"],
            parameters["admittance_damping_ns_m"],
            parameters["admittance_stiffness_n_m"],
        )
        self._kinematics = CrankSliderKinematics(
            **{
                key.removeprefix("geometry_"): value
                for key, value in parameters.items()
                if key.startswith("geometry_")
            }
        )
        self.published = []
        self._mit_pub = SimpleNamespace(publish=self.published.append)
        self.inputs = None

    def get_parameter(self, key: str) -> SimpleNamespace:
        """模拟 ROS 参数读取，数值直接来自现役 YAML。"""
        return SimpleNamespace(value=self.parameters[key])

    def _fresh_inputs(self) -> tuple:
        """返回本周期已验证的观测；输入保护由 ROS safety 测试覆盖。"""
        return self.inputs

    def _log_runtime_state(self, *args: object) -> None:
        """离线回放省略日志输出。"""

    def _trip(self, reason: str) -> None:
        """任何意外故障都使回放失败，不能静默吞掉异常。"""
        raise AssertionError(reason)


@pytest.mark.parametrize("periods", [(0.002,), (0.004,), (0.002, 0.003, 0.008, 0.004)])
def test_ros_and_simulation_emit_equal_tracking_requests(periods, monkeypatch) -> None:
    """两个真实适配路径对相同观测与复位产生一致的五字段 MIT 请求和积分状态。"""
    ros_config = Path(ros_module.__file__).resolve().parents[1] / "config/dm_force_tracking.yaml"
    parameters = yaml.safe_load(ros_config.read_text())["dm_force_tracking"]["ros__parameters"]
    profile = compose_research_run(
        experiment="dm_gripper/force_tracking_admittance",
        overrides=("seed=20260814", "execution=plan"),
    ).profile
    simulator = DMAdmittanceController(_ReplayMotor(), profile.normal_force, profile.mit)
    node = _ROSReplay(parameters)
    assert simulator.command_config == node._command_config()
    simulator.state = "force_tracking"
    simulator.trajectory = object()
    simulator.reference_position_rad = 0.5
    now_s = [0.0]
    monkeypatch.setattr(ros_module.time, "monotonic", lambda: now_s[0])
    for index in range(180):
        if index == 90:
            simulator.admittance.reset()
            node._admittance.reset()
        previous_s = now_s[0]
        now_s[0] += periods[index % len(periods)]
        dt_s = now_s[0] - previous_s
        motor = SimpleNamespace(
            position_rad=0.5 + 0.003 * math.sin(index / 20),
            velocity_rad_s=0.02 * math.cos(index / 20),
            enabled=True,
            control_mode=ros_module.DmMotorState.MODE_MIT,
        )
        left = SimpleNamespace(normal_force_n=0.5 + 0.12 * math.sin(index / 11), data_valid=True)
        right = SimpleNamespace(normal_force_n=0.5 + 0.10 * math.cos(index / 13), data_valid=True)
        node.inputs = (motor, left, right)
        node._control_tick()
        simulator.step(
            motor,
            observation=ForceControlObservation(
                time_s=now_s[0],
                approach_position=0.5,
                total_normal_force_n=left.normal_force_n + right.normal_force_n,
                left_normal_force_n=left.normal_force_n,
                right_normal_force_n=right.normal_force_n,
                dt=dt_s,
            ),
            reference=ForceControlReference(target_force_n=parameters["target_normal_force_n"]),
        )
        assert len(node.published) == index + 1
        expected = asdict(simulator.last_requested_command)
        actual = {name: getattr(node.published[-1], name) for name in expected}
        assert actual == pytest.approx(expected, rel=0, abs=1e-12)
        assert node._admittance.displacement_m == simulator.admittance.displacement_m
        assert node._admittance.velocity_m_s == simulator.admittance.velocity_m_s
