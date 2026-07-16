"""验证统一触觉帧、CSV 与 Rerun 记录钩子。"""

from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


def _load_recording():
    path = Path(__file__).resolve().parents[1] / "scripts/recording.py"
    spec = importlib.util.spec_from_file_location("recording", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _frame(module, step: int = 0):
    left = np.arange(27, dtype=float).reshape(3, 3, 3)
    right = left + 100
    return module.TactileFrame(step, step * 0.002, 20 + step, left, right)


def test_tactile_frame_validates_grid_and_sums_full_force() -> None:
    module = _load_recording()
    frame = _frame(module)
    np.testing.assert_allclose(frame.left_force, frame.left.sum(axis=(1, 2)))
    np.testing.assert_allclose(frame.right_force, frame.right.sum(axis=(1, 2)))
    with pytest.raises(ValueError, match=r"\(3, rows, cols\)"):
        module.TactileFrame(0, 0.0, 0.0, np.zeros((2, 3, 3)), np.zeros((3, 3, 3)))


def test_force_csv_recorder_writes_downsampled_samples(tmp_path: Path) -> None:
    module = _load_recording()
    output = tmp_path / "forces.csv"
    recorder = module.ForceCsvRecorder(output, every=2)
    recorder.record(_frame(module, step=0))
    recorder.record(_frame(module, step=1))
    recorder.close()
    with output.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    assert len(rows) == 1
    assert rows[0]["step"] == "0"
    assert float(rows[0]["left_fx"]) == 36.0
    assert float(rows[0]["left_fz"]) == 198.0
    assert float(rows[0]["right_fz"]) == 1098.0


@pytest.mark.parametrize(("shape", "display_shape"), [((3, 3), (3, 3)), ((32, 32), (8, 8))])
def test_aggregate_shear_preserves_force(shape, display_shape) -> None:
    module = _load_recording()
    rows, cols = shape
    tactile = np.arange(3 * rows * cols, dtype=float).reshape(3, rows, cols)
    origins, vectors, pressures = module.aggregate_shear(tactile)
    assert len(origins) == display_shape[0] * display_shape[1]
    np.testing.assert_allclose(vectors.sum(axis=0), tactile[:2].sum(axis=(1, 2)))
    np.testing.assert_allclose(pressures.sum(), tactile[2].sum())


def test_shear_grid_lines_bound_each_display_cell() -> None:
    module = _load_recording()
    lines = module._shear_grid_lines(3, 4)
    assert len(lines) == 9
    np.testing.assert_allclose(lines[0], ((-0.5, -0.5), (-0.5, 2.5)))
    np.testing.assert_allclose(lines[4], ((3.5, -0.5), (3.5, 2.5)))
    np.testing.assert_allclose(lines[-1], ((-0.5, 2.5), (3.5, 2.5)))


def test_rerun_logger_uses_simulation_timelines_and_full_tensors(
    tmp_path: Path, monkeypatch
) -> None:
    module = _load_recording()
    import rerun as rr

    class FakeRecording:
        def __init__(self, _application_id):
            self.logs = []
            self.times = []
            self.sinks = None
            self.disconnected = False

        def set_sinks(self, *sinks, **_kwargs):
            self.sinks = sinks

        def log(self, path, value, *, static=False):
            self.logs.append((path, value, static))

        def set_time(self, timeline, **value):
            self.times.append((timeline, value))

        def flush(self):
            return None

        def disconnect(self):
            self.disconnected = True

    fake_recording = FakeRecording("unused")
    monkeypatch.setattr(rr, "RecordingStream", lambda _application_id: fake_recording)
    monkeypatch.setattr(rr, "FileSink", lambda path: ("file", path))
    monkeypatch.setattr(module, "_rerun_blueprint", lambda: object())

    logger = module.RerunTactileLogger(
        "test", live=False, path=tmp_path / "sample.rrd", hz=100, pressure_max=15
    )
    fake_recording.logs.clear()
    logger.record(_frame(module, step=0))
    fake_recording.logs.clear()
    fake_recording.times.clear()
    logger.record(_frame(module, step=1))
    assert fake_recording.logs == []
    logger.record(_frame(module, step=5))

    assert fake_recording.times == [
        ("step", {"sequence": 5}),
        ("sim_time", {"duration": 0.01}),
    ]
    paths = [path for path, _value, _static in fake_recording.logs]
    assert "control/value" in paths
    assert "tactile/left/raw" in paths
    assert "tactile/right/raw" in paths
    assert "tactile/left/shear" in paths
    assert not any("force_3d" in path for path in paths)
    left_pressure = next(
        value for path, value, _static in fake_recording.logs if path == "tactile/left/pressure"
    )
    value_range = left_pressure.as_component_batches()[1].as_arrow_array().to_pylist()
    assert value_range == [[0.0, 15.0]]
    logger.close()
    assert fake_recording.disconnected

    fake_recording.logs.clear()
    fake_recording.times.clear()
    fake_recording.disconnected = False
    logger = module.RerunTactileLogger("test", live=False, path=tmp_path / "60hz.rrd", hz=60)
    for step in range(51):
        logger.record(_frame(module, step=step))
    logged_steps = [
        value["sequence"]
        for timeline, value in fake_recording.times
        if timeline == "step"
    ]
    assert logged_steps == [0, 9, 17, 25, 34, 42, 50]
    logger.close()
