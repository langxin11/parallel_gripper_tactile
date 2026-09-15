"""离线回放只保留一项端到端契约，不复制策略测试矩阵。"""

import csv
import json

import pytest

from dm_grasp_core.grasp.unified import UnifiedAdaptiveConfig
from dmgripper_experiments.replay import replay_tactile


def test_replay_is_diagnostic_deduplicated_and_exclusive(tmp_path) -> None:
    """回放强制旁路、重复不积分，并拒绝覆盖输入或既有结果。"""
    source, destination = tmp_path / "tactile.jsonl", tmp_path / "diagnostic.csv"
    samples = [
        {
            "timestamp_us": timestamp,
            "left_force_n": 0.9,
            "right_force_n": 0.9,
            "left_taxel_forces_n": [[0.1, 0, 0.1]] * 9,
            "right_taxel_forces_n": [[0.1, 0, 0.1]] * 9,
        }
        for timestamp in (0, 10000, 10000, 20000)
    ]
    source.write_text("\n".join(json.dumps(sample) for sample in samples), encoding="utf-8")
    config = UnifiedAdaptiveConfig(risk_enabled=True, friction_update_enabled=True)
    assert replay_tactile(source, destination, config) == 3
    with destination.open() as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 3
    assert all(row["diagnostic_only"] == "True" for row in rows)
    assert all(float(row["adaptive_left_mu"]) == 0.6 for row in rows)
    assert float(rows[0]["raw_load_requirement_n"]) == pytest.approx(2.25)
    for output in (source, destination):
        with pytest.raises(FileExistsError):
            replay_tactile(source, output, config)
