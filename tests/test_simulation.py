"""针对单所有者仿真会话的调度测试。"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import pytest

from parallel_gripper_tactile.simulation import Experiment, SimulationSession
from parallel_gripper_tactile.simulation.session import (
    Experiment as SessionExperiment,
)
from parallel_gripper_tactile.simulation.session import (
    SimulationSession as SessionSimulationSession,
)


def test_public_simulation_imports_reexport_session_objects() -> None:
    """稳定入口与新实现路径导出同一对象。"""
    assert Experiment is SessionExperiment
    assert SimulationSession is SessionSimulationSession


def _model() -> mujoco.MjModel:
    """编译一个已知固定时间步长的最小自由 MuJoCo 模型。"""
    return mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <option timestep="0.001" gravity="0 0 0" />
          <worldbody><body name="body" pos="0 0 0"><freejoint/><geom size="0.01" /></body></worldbody>
        </mujoco>
        """
    )


@dataclass
class _SpyExperiment:
    """记录生命周期调用，同时向断言暴露采样状态。"""

    session: SimulationSession
    before_times: list[float] = field(default_factory=list)
    control_dts: list[float] = field(default_factory=list)
    sampled_times: list[float] = field(default_factory=list)

    def before_step(self, time_s: float) -> None:
        """记录积分前的时间。"""
        self.before_times.append(time_s)

    def control(self, dt_s: float) -> None:
        """记录独立调度的控制更新。"""
        self.control_dts.append(dt_s)

    def sample(self) -> float:
        """记录并返回积分后的采样时间。"""
        time_s = float(self.session.data.time)
        self.sampled_times.append(time_s)
        return time_s


def test_session_owns_step_order_and_decouples_control_and_sample_periods() -> None:
    """生命周期顺序遵循独立时钟下的 before/control/physics/sample。"""
    model = _model()
    session = SimulationSession(
        model,
        mujoco.MjData(model),
        control_period_s=0.002,
        sample_period_s=0.003,
    )
    experiment = _SpyExperiment(session)

    samples = session.run_steps(7, experiment)

    assert session.steps == 7
    assert session.data.time == pytest.approx(0.007)
    assert experiment.before_times == pytest.approx([0.0, 0.001, 0.002, 0.003, 0.004, 0.005, 0.006])
    assert experiment.control_dts == pytest.approx([0.002, 0.002, 0.002, 0.002])
    assert samples == pytest.approx([0.001, 0.003, 0.006])
    assert experiment.sampled_times == pytest.approx(samples)


def test_advance_to_crosses_an_unaligned_target_and_forwards_samples() -> None:
    """推进到目标时间时使用固定的物理边界与采样回调。"""
    model = _model()
    session = SimulationSession(model, mujoco.MjData(model), sample_period_s=0.002)
    experiment = _SpyExperiment(session)
    observed: list[float] = []

    samples = session.advance_to(0.0032, experiment, on_sample=observed.append)

    assert session.steps == 4
    assert session.data.time == pytest.approx(0.004)
    assert samples == pytest.approx([0.001, 0.002, 0.004])
    assert observed == pytest.approx(samples)
    with pytest.raises(ValueError, match="must not precede"):
        session.advance_to(0.003, experiment)


def test_session_rejects_impossible_or_invalid_periods() -> None:
    """比物理更快的时钟与负步数运行都是显式错误。"""
    model = _model()
    with pytest.raises(ValueError, match="must not be shorter"):
        SimulationSession(model, mujoco.MjData(model), control_period_s=0.0005)
    with pytest.raises(ValueError, match="finite and positive"):
        SimulationSession(model, mujoco.MjData(model), sample_period_s=0.0)

    session = SimulationSession(model, mujoco.MjData(model))
    experiment = _SpyExperiment(session)
    with pytest.raises(ValueError, match="nonnegative"):
        session.run_steps(-1, experiment)
