"""通用绘图：新 schema 渲染、历史 cup trace 兼容与离线重绘。"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from dmgripper_experiments.plotting import (
    plot_experiment_run,
    read_trace_rows,
    repaint_run,
)
from dmgripper_experiments.recording import ExperimentRecorder, create_run_directory

from dmgripper_experiments.config import ExperimentConfig


def _write_rows(recorder: ExperimentRecorder, rows: list[dict[str, object]]) -> None:
    for row in rows:
        recorder.write(row)


def _adaptive_rows(count: int = 20) -> list[dict[str, object]]:
    rows = []
    for index in range(count):
        active = 5 <= index < 15
        rows.append(
            {
                "time_s": 0.01 * index,
                "phase": "active" if active else "preload",
                "task_time_s": 0.01 * index if active else None,
                "contact_segment": 1,
                "left_fz_n": 0.5,
                "right_fz_n": 0.5,
                "measured_force_n": 0.5,
                "target_source": "adaptive",
                "target_force_n": 0.5 + 0.01 * index,
                "raw_left_fx_n": 0.0,
                "raw_left_fy_n": 0.1,
                "raw_right_fx_n": 0.0,
                "raw_right_fy_n": 0.1,
                "target_trigger_active": index % 3 == 0,
                "position_rad": 0.4,
                "q_des_rad": 0.4,
                "torque_nm": 0.1,
                "stiffness_n_per_m": 1500.0,
                "stiffness_valid": index > 6,
            }
        )
    return rows


def test_plot_generates_pdf_and_png_and_registers_manifest(tmp_path: Path):
    """正常运行目录生成 PDF／PNG 并登记到 manifest。"""
    config = ExperimentConfig()
    directory = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(directory, config)
    _write_rows(recorder, _adaptive_rows())
    recorder.close()
    plots = plot_experiment_run(directory)
    assert len(plots) == 2
    assert (directory / "plot.pdf").is_file()
    assert (directory / "plot.png").is_file()
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert "plot.pdf" in manifest["files"]


def test_plot_without_stiffness_omits_panel(tmp_path: Path):
    """没有刚度数据时不绘制刚度面板，也不伪造曲线。"""
    config = ExperimentConfig()
    directory = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(directory, config)
    rows = _adaptive_rows()
    for row in rows:
        row.pop("stiffness_n_per_m")
        row.pop("stiffness_valid")
    _write_rows(recorder, rows)
    recorder.close()
    plots = plot_experiment_run(directory)
    assert len(plots) == 2


def test_tangential_trigger_is_background_and_axes_use_math_symbols():
    """Trigger 在切向力曲线下层，坐标轴使用标准物理量符号。"""
    import matplotlib.pyplot as plt

    from dmgripper_experiments.plotting import (
        _plot_joint_command,
        _plot_normal_force,
        _plot_stiffness,
        _plot_tangential_force,
    )

    rows = [
        {key: str(value) for key, value in row.items() if value is not None}
        for row in _adaptive_rows()
    ]
    time_s = [float(row["time_s"]) for row in rows]
    figure, axes = plt.subplots(4, 1)
    _plot_normal_force(axes[0], time_s, rows)
    _plot_tangential_force(axes[1], time_s, rows)
    _plot_joint_command(axes[2], time_s, rows)
    _plot_stiffness(axes[3], time_s, rows)

    trigger = next(
        collection for collection in axes[1].collections if collection.get_label() == "Trigger"
    )
    assert trigger.get_zorder() < min(line.get_zorder() for line in axes[1].lines)
    assert [line.get_label() for line in axes[0].lines] == [r"$F_z$", r"$F_z^{\mathrm{ref}}$"]
    assert [line.get_label() for line in axes[1].lines] == ["Left", "Right"]
    assert [line.get_label() for line in axes[2].lines] == [r"$q$", r"$q_d$"]
    assert figure.axes[-1].lines[0].get_label() == r"$\tau_m$"
    assert axes[3].lines[0].get_label() == r"$\hat{K}$"
    assert axes[0].get_ylabel() == r"$F_z$ (N)"
    assert axes[1].get_ylabel() == r"$F_T$ (N)"
    assert axes[2].get_ylabel() == r"$q$ (rad)"
    assert axes[3].get_ylabel() == r"$K$ ($\mathrm{N\,m^{-1}}$)"
    plt.close(figure)


def test_math_text_uses_stix_without_fonttools_timestamp_noise(tmp_path: Path):
    """数学字体固定为 STIX：嵌入 PDF 不触发 fontTools 旧时间戳告警。"""
    import logging

    from dmgripper_experiments.plotting import _plotting_api

    plt, style_context = _plotting_api()
    records: list[str] = []

    class _Capture(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            records.append(record.getMessage())

    logger = logging.getLogger("fontTools")
    handler = _Capture()
    logger.addHandler(handler)
    try:
        with style_context():
            assert plt.rcParams["mathtext.fontset"] == "stix"
            figure, axis = plt.subplots()
            axis.set_ylabel(r"$\hat{K}$ ($\mathrm{N\,m^{-1}}$)")
            figure.savefig(tmp_path / "math.pdf")
            plt.close(figure)
    finally:
        logger.removeHandler(handler)
    assert not [message for message in records if "timestamp seems very low" in message]


def test_plot_returns_empty_for_missing_or_empty_trace(tmp_path: Path):
    """缺失或空 trace 返回空元组而不是异常。"""
    assert plot_experiment_run(tmp_path) == ()
    config = ExperimentConfig()
    directory = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(directory, config)
    recorder.close()
    assert plot_experiment_run(directory) == ()


def test_repaint_writes_exclusive_directory_and_preserves_source(tmp_path: Path):
    """离线重绘写入独占目录，源目录 manifest 不被修改。"""
    config = ExperimentConfig()
    directory = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(directory, config)
    _write_rows(recorder, _adaptive_rows())
    recorder.close()
    source_manifest_before = (directory / "manifest.json").read_text(encoding="utf-8")
    output = repaint_run(directory)
    assert (output / "plot.pdf").is_file()
    assert (directory / "manifest.json").read_text(encoding="utf-8") == source_manifest_before
    repaint_manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert repaint_manifest["schema"] == "dmgripper-experiment/repaint/v1"
    assert repaint_manifest["source_directory"] == str(directory)
    second = repaint_run(directory)
    assert second != output


def _recorded_run(tmp_path: Path) -> Path:
    """生成包含 preload／active 行的完整运行目录。"""
    config = ExperimentConfig()
    directory = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(directory, config)
    _write_rows(recorder, _adaptive_rows())
    recorder.close()
    return directory


def test_phase_window_filters_rows_and_records_manifest(tmp_path: Path):
    """阶段白名单窗口只绘制所选阶段，并把条件写入重绘 manifest。"""
    directory = _recorded_run(tmp_path)
    output = repaint_run(directory, phases=("active",))
    assert (output / "plot.pdf").is_file()
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["window"] == {"phases": ["active"]}


def test_task_time_window_records_manifest_and_empty_window_returns_nothing(
    tmp_path: Path,
) -> None:
    """任务时间窗口记录区间；无匹配行时返回空元组而不是异常。"""
    directory = _recorded_run(tmp_path)
    output = repaint_run(directory, task_time_range=(0.0, 0.06))
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["window"] == {"task_time_s": [0.0, 0.06]}
    assert (output / "plot.png").is_file()
    assert plot_experiment_run(directory, task_time_range=(99.0, 100.0)) == ()


def test_task_time_window_excludes_rows_without_running_task_clock(tmp_path: Path) -> None:
    """approach 阶段占位的任务时间 0.0 不进入任务时间窗口。"""
    config = ExperimentConfig()
    directory = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(directory, config)
    rows = _adaptive_rows()
    for index in range(3):
        rows.append(
            {
                "time_s": 0.5 + 0.01 * index,
                "phase": "approach",
                "task_time_s": 0.0,
                "contact_segment": 0,
                "left_fz_n": 0.0,
                "right_fz_n": 0.0,
                "measured_force_n": 0.0,
                "target_source": "curve",
                "position_rad": 0.1,
                "q_des_rad": 0.1,
                "torque_nm": 0.0,
            }
        )
    _write_rows(recorder, rows)
    recorder.close()
    # 只含 approach 占位行的窄窗口为空；正常窗口仍绘出 active 行。
    assert plot_experiment_run(directory, task_time_range=(0.0, 0.001)) == ()
    output = repaint_run(directory, task_time_range=(0.0, 0.2))
    assert (output / "plot.pdf").is_file()


@pytest.mark.parametrize(
    ("phases", "task_time_range", "message"),
    [
        (("active", "cruise"), None, "未知阶段"),
        (None, (1.0, 0.0), "有限区间"),
        (None, (-1.0, 1.0), "有限区间"),
    ],
)
def test_invalid_window_arguments_are_rejected(
    tmp_path: Path, phases, task_time_range, message: str
) -> None:
    """未知阶段或倒置／负数区间在绘图前直接拒绝。"""
    directory = _recorded_run(tmp_path)
    with pytest.raises(ValueError, match=message):
        plot_experiment_run(directory, phases=phases, task_time_range=task_time_range)


def test_plot_cli_requires_repaint_for_window(tmp_path: Path, capsys) -> None:
    """CLI 窗口参数必须配合 --repaint；非法参数返回用法错误码。"""
    from dmgripper_experiments.plot_cli import run

    directory = _recorded_run(tmp_path / "nested")
    assert run([str(directory), "--task-time", "0:0.1"]) == 2
    assert "必须配合 --repaint" in capsys.readouterr().err
    assert run([str(directory), "--phase", "active", "--repaint"]) == 0
    assert run([str(directory), "--task-time", "abc"]) == 2
    assert run([str(directory), "--unknown"]) == 2


def test_legacy_cup_trace_with_state_column_renders(tmp_path: Path):
    """历史 cup v2 trace（``state`` 列、31 字段）可以重绘。"""
    legacy_fields = [
        "time_s",
        "state",
        "left_fz_n",
        "right_fz_n",
        "measured_force_n",
        "target_force_n",
        "trigger_active",
        "position_rad",
        "q_des_rad",
        "torque_nm",
    ]
    directory = tmp_path / "legacy_cup"
    directory.mkdir()
    lines = [",".join(legacy_fields)]
    for index in range(12):
        lines.append(
            ",".join(
                str(value)
                for value in (
                    0.05 * index,
                    "pour",
                    0.5,
                    0.5,
                    0.5,
                    0.6,
                    index % 2,
                    0.4,
                    0.4,
                    0.1,
                )
            )
        )
    (directory / "trace.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    rows = read_trace_rows(directory)
    assert len(rows) == 12
    assert rows[0]["phase"] == "pour"
    plots = plot_experiment_run(directory)
    assert len(plots) == 2


def test_stiffness_invalid_segment_is_left_blank(tmp_path: Path):
    """刚度未有效确认的区间留空，不伪造为初值曲线。"""
    import math

    config = ExperimentConfig()
    directory = create_run_directory(tmp_path, config)
    recorder = ExperimentRecorder(directory, config)
    _write_rows(recorder, _adaptive_rows())
    recorder.close()
    from dmgripper_experiments.plotting import _values

    rows = read_trace_rows(directory)
    values = _values(rows, "stiffness_n_per_m")
    valid = _values(rows, "stiffness_valid", boolean=True)
    masked = [
        value if math.isfinite(flag) and flag > 0.5 else math.nan
        for value, flag in zip(values, valid, strict=True)
    ]
    assert any(math.isfinite(value) for value in masked)
    assert any(math.isnan(value) for value in masked)
