"""用少量合成力序列验证候选抑制、事件生命周期和输入边界。"""

from dataclasses import replace

import numpy as np
import pytest

from dm_grasp_core.tactile.risk import TaxelRiskConfig, TaxelRiskObserver


def _frame(time_s: float, mode: str = "risk") -> np.ndarray:
    """生成可控制剪切重分布、加载趋势和法向增力的九点数据。"""
    frame = np.zeros((9, 3))
    frame[:, 2] = 0.2 + (time_s if mode == "normal" else 0)
    load = (1.0 if mode == "local_release" else 0.3) + (0 if mode == "stopped" else time_s)
    weights = np.full(9, 0.02)
    shift = 0 if mode == "uniform" else min(0.3, 0.5 * time_s)
    weights[0], weights[1] = 0.43 - shift, 0.43 + shift
    frame[:, 0] = load * weights
    return frame


@pytest.mark.parametrize(
    "mode,expected",
    [
        ("risk", True),
        ("local_release", True),
        ("normal", False),
        ("uniform", False),
        ("stopped", False),
    ],
)
def test_evidence_requires_loading_and_redistribution(mode: str, expected: bool) -> None:
    """同样的局部变化在停止加载或主动增力时不构成闭环候选。"""
    observer = TaxelRiskObserver()
    events = []
    for time in np.arange(0, 0.65, 0.01):
        observation = observer.update(_frame(time, mode), _frame(time, mode), time_s=float(time))
        if observation.event_id:
            events.append(observation)
    assert bool(events) is expected
    if expected:
        assert len(events) == 1
        if mode == "local_release":
            assert 1 < events[0].left_candidate < 3
            assert events[0].left_quality == 1
        else:
            # 剪切重分配已满足风险门槛，但局部力比仍增长，不授予摩擦候选。
            assert events[0].left_candidate is None
            assert events[0].left_quality == 0


def test_timestamp_contact_rearming_and_reset() -> None:
    """重复与回退不重发，拓扑重建窗口，重新加载与显式重置保留事件唯一性。"""
    observer = TaxelRiskObserver()
    events = []
    for index in range(180):
        time = index * 0.01
        local_time = time if time < 0.65 else max(0, time - 1.0)
        mode = "risk" if time < 0.65 or time >= 1 else "stopped"
        result = observer.update(_frame(local_time, mode), _frame(local_time, mode), time_s=time)
        if result.event_id:
            events.append(result.event_id)
            duplicate = observer.update(_frame(local_time), _frame(local_time), time_s=time)
            assert duplicate.event_id == 0 and duplicate.left_candidate is None
    assert events == [1, 2]
    with pytest.raises(ValueError, match="回退"):
        observer.update(_frame(0), _frame(0), time_s=0)
    changed = _frame(0)
    changed[0, 2] = 0
    result = observer.update(changed, changed, time_s=1.8)
    assert result.contact_changed and result.event_id == 0
    assert not observer.update(changed, changed, time_s=1.81).contact_changed
    observer.reset()
    for time in np.arange(0, 0.65, 0.01):
        result = observer.update(_frame(time), _frame(time), time_s=float(time))
        if result.event_id:
            assert result.event_id == 3
            break
    else:
        pytest.fail("重建后应能确认新事件")


def test_invalid_data_and_configuration_rebuild_window() -> None:
    """饱和、非有限值、无接触和长间断不能沿用旧窗口。"""
    for changes in (
        {"window_s": True},
        {"window_s": "0.2"},
        {"max_gap_s": float("nan")},
        {"release_ratio": 1},
        {"min_normal_n": 15},
    ):
        with pytest.raises(ValueError):
            replace(TaxelRiskConfig(), **changes)
    observer = TaxelRiskObserver()
    for time in np.arange(0, 0.2, 0.01):
        observer.update(_frame(time), _frame(time), time_s=float(time))
    for index, value in enumerate((4.0, float("nan"), float("inf"))):
        bad = _frame(0.2)
        bad[0, 0] = value
        result = observer.update(bad, _frame(0.2), time_s=0.2 + index * 0.01)
        assert not result.valid and not result.left_valid_mask[0] and result.event_id == 0
    result = observer.update(_frame(0.3), _frame(0.3), time_s=0.3)
    assert result.valid and result.event_id == 0
    result = observer.update(_frame(0.5), _frame(0.5), time_s=0.5)
    assert result.reason == "warming_up" and result.event_id == 0
    result = observer.update(np.zeros((9, 3)), _frame(0.5), time_s=0.51)
    assert not result.valid
