"""采集线程状态超前于控制快照时的逐触点事件边界回归。"""

import pytest

from .test_native_slip import _estimates, _running, _update


@pytest.mark.parametrize(
    "old_fields",
    [
        {"native_session_phase": "starting"},
        {"native_session_id": 0},
        {"native_slip_active": ()},
    ],
)
def test_ineligible_snapshot_does_not_consume_next_valid_slipped_event(old_fields) -> None:
    """工作线程已活动时，旧阶段或旧会话快照不能吞掉下一有效包的首个滑移。"""
    session, worker = _running()
    states = ((3, 1), (3, 1))

    ignored = _update(session, worker, 0.5, native_pillar_states=states, **old_fields)
    accepted = _update(session, worker, 0.625, native_pillar_states=states)

    assert _estimates(ignored) == []
    assert [(item["side"], item["pillar_id"]) for item in _estimates(accepted)] == [
        ("left", 0),
        ("right", 0),
    ]
    assert worker.stops[-1] == "estimates_ready"
