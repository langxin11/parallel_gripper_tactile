"""验证 waypoint 目标力跟踪任务。"""

import csv
import math
from pathlib import Path

import pyarrow.parquet as pq
import pytest
from PIL import Image
from parallel_gripper_tactile.experiments import force_tracking as force_tracking_module

from parallel_gripper_tactile.experiments.force_tracking import (
    ForceReference,
    ForceTrackingTask,
    ForceWaypoint,
    _downsample_force_tracking_rows,
    _evaluate_tracking,
    _force_tracking_plot_layout,
    _plot_force_tracking,
    configure_force_controller,
    run_force_tracking,
)
from parallel_gripper_tactile.config.profiles import AdrcControl, TorqueAdrcControl, load_profile


ROOT = Path(__file__).resolve().parents[1]


def test_force_reference_waypoints_interpolate_targets() -> None:
    """waypoint 目标力曲线支持保持、线性和平滑插值。"""
    waypoints = (ForceWaypoint(t_s=0.0, force_n=2.0), ForceWaypoint(t_s=1.0, force_n=6.0))

    assert ForceReference(interpolation="hold", waypoints=waypoints).target_at(0.5) == 2.0
    assert ForceReference(interpolation="linear", waypoints=waypoints).target_at(0.5) == 4.0
    assert ForceReference(interpolation="smoothstep", waypoints=waypoints).target_at(0.5) == 4.0
    assert ForceReference(interpolation="linear", waypoints=waypoints).target_at(2.0) == 6.0


def test_force_tracking_task_loads_default_waypoint_config() -> None:
    """默认动态力跟踪任务来自 YAML 配置文件。"""
    task = ForceTrackingTask.load(ROOT / "configs/force_tracking/default_waypoints.yaml")

    assert task.name == "default_waypoint_force_tracking"
    assert task.approach.feedforward_force_n == pytest.approx(1.0)
    assert task.reference.duration_s == pytest.approx(4.5)
    assert task.reference.target_at(3.5) == pytest.approx(10.0)


def _plot_rows() -> list[dict[str, float | str]]:
    """构造覆盖三个任务感知布局的轻量 trace。"""
    rows: list[dict[str, float | str]] = []
    for index in range(21):
        time_s = index * 0.05
        target = 2.0 + 4.0 * min(time_s, 0.5)
        filtered = target - 0.15 * math.sin(2.0 * math.pi * time_s)
        rows.append(
            {
                "time_s": time_s,
                "tracking_time_s": time_s,
                "phase": "track_reference",
                "force_semantics": "average_side",
                "target_normal_force_n": target,
                "filtered_normal_force_n": filtered,
                "measured_normal_force_n": filtered + 0.05,
                "tracking_error_n": target - filtered,
                "motor_torque_n_m": 0.02 * target,
                "estimated_contact_stiffness_n_per_m": 100.0 + 10.0 * target,
            }
        )
    return rows


@pytest.mark.parametrize(
    ("interpolation", "expected_layout"),
    [
        ("hold", "step_transient"),
        ("linear", "ramp_hysteresis"),
        ("smoothstep", "waypoint_error"),
    ],
)
def test_force_tracking_plot_is_task_aware_and_writes_format_pair(
    tmp_path: Path, interpolation: str, expected_layout: str, fast_plot_render: None
) -> None:
    """三类任务分别选择瞬态、滞后和 waypoint 误差诊断，并输出 PNG/PDF。"""
    task = ForceTrackingTask(
        schema_version=1,
        name=f"{interpolation}_plot",
        reference=ForceReference(
            interpolation=interpolation,  # type: ignore[arg-type]
            waypoints=(
                ForceWaypoint(t_s=0.0, force_n=2.0),
                ForceWaypoint(t_s=0.5, force_n=4.0),
                ForceWaypoint(t_s=1.0, force_n=3.0),
            ),
        ),
    )
    output = tmp_path / f"{interpolation}.png"

    assert _force_tracking_plot_layout(task) == expected_layout
    _plot_force_tracking(output, _plot_rows(), task=task)

    assert output.is_file()
    assert output.with_suffix(".pdf").is_file()
    assert output.stat().st_size > 0
    assert output.with_suffix(".pdf").stat().st_size > 0


def test_force_tracking_publication_plot_preserves_high_resolution(tmp_path: Path) -> None:
    """代表性 Step 图应保留跨栏宽度和 600 DPI 出版契约。"""
    task = ForceTrackingTask(
        schema_version=1,
        name="publication_plot",
        reference=ForceReference(
            interpolation="hold",
            waypoints=(
                ForceWaypoint(t_s=0.0, force_n=2.0),
                ForceWaypoint(t_s=0.5, force_n=4.0),
            ),
        ),
    )
    output = tmp_path / "publication.png"

    _plot_force_tracking(output, _plot_rows(), task=task)

    with Image.open(output) as image:
        assert image.size[0] >= 4_000
        assert image.info["dpi"][0] == pytest.approx(600.0, abs=0.1)


def test_force_tracking_trace_downsampling_preserves_events_and_boundaries() -> None:
    """降采样保留首末、状态变化及 hold 阶跃邻域的全频行。"""
    task = ForceTrackingTask(
        schema_version=1,
        name="sampled_hold",
        control_period_s=0.01,
        reference=ForceReference(
            interpolation="hold",
            waypoints=(
                ForceWaypoint(t_s=0.0, force_n=2.0),
                ForceWaypoint(t_s=0.5, force_n=5.0),
                ForceWaypoint(t_s=1.0, force_n=3.0),
            ),
        ),
    )
    rows = [
        {
            "time_s": index * 0.01,
            "tracking_time_s": index * 0.01,
            "phase": "track_reference",
            "control_state": "force_tracking" if index < 30 else "settled",
            "torque_adrc_rate_limited": "false" if index < 70 else "true",
            "torque_adrc_amplitude_limited": "false",
        }
        for index in range(101)
    ]

    sampled = _downsample_force_tracking_rows(
        rows,
        task=task,
        trace_sample_period_s=0.1,
        trace_event_window_s=0.02,
    )

    assert len(sampled) < len(rows)
    assert sampled[0] == rows[0]
    assert sampled[-1] == rows[-1]
    assert all(rows[index] in sampled for index in (29, 30, 69, 70))
    assert all(rows[index] in sampled for index in (49, 50, 51))


@pytest.mark.parametrize(
    (
        "variant",
        "enabled",
        "position_gain",
        "torque_gain",
        "feedback_gain",
        "position_limit_enabled",
    ),
    [
        ("pid-only", False, 0.25, 1.0, 0.0, False),
        ("pid-torque-ff", True, 0.0, 1.0, 0.0, False),
        ("pid-stiffness-ff", True, 0.25, 0.0, 0.0, False),
        ("pid-stiffness-limit", True, 0.0, 1.0, 0.0, True),
        ("full", True, 0.25, 1.0, 0.0, False),
        ("direct-torque", True, 0.0, 1.0, 1.0, False),
        ("adrc", True, 0.0, 1.0, 0.0, False),
        ("adrc-torque", True, 0.0, 1.0, 0.0, False),
        ("adrc-torque-td", True, 0.0, 1.0, 0.0, False),
    ],
)
def test_controller_variants_apply_reproducible_ablation_settings(
    variant: str,
    enabled: bool,
    position_gain: float,
    torque_gain: float,
    feedback_gain: float,
    position_limit_enabled: bool,
) -> None:
    """各控制器档位只修改对应的刚度估计与前馈开关。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    configured = configure_force_controller(  # type: ignore[arg-type]
        source, variant=variant, sensor_noise_seed=17
    )

    assert configured.normal_force is not None
    assert configured.normal_force.sensor_noise_seed == 17
    assert configured.normal_force.torque_feedback_gain == feedback_gain
    assert configured.normal_force.stiffness is not None
    assert configured.normal_force.stiffness.enabled is enabled
    assert configured.normal_force.stiffness.position_feedforward_gain == position_gain
    assert configured.normal_force.stiffness.torque_feedforward_gain == torque_gain
    assert configured.normal_force.stiffness.position_limit_enabled is position_limit_enabled
    assert source.normal_force is not None
    assert source.normal_force.sensor_noise_seed == 20260814
    assert source.normal_force.torque_feedback_gain == 0.0


def test_direct_torque_variant_keeps_mit_gains_for_approach_servo() -> None:
    """direct-torque 不在 profile 层清零 MIT kp/kd，接近阶段保持位置伺服。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    configured = configure_force_controller(source, variant="direct-torque")

    assert source.mit is not None
    assert configured.mit is not None
    assert configured.mit.kp == source.mit.kp
    assert configured.mit.kd == source.mit.kd
    assert configured.normal_force is not None
    assert configured.normal_force.torque_feedback_gain == 1.0


def test_adrc_variant_enables_ladrc_outer_loop_with_default_parameters() -> None:
    """adrc 变体注入默认 LADRC 参数，位置前馈置 0，力矩前馈与 MIT 增益不动。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    configured = configure_force_controller(source, variant="adrc")

    assert configured.normal_force is not None
    # LADRC 外环以代码与 AdrcControl 默认值生效，profile 无需显式配置。
    assert configured.normal_force.adrc is not None
    assert configured.normal_force.adrc == AdrcControl()
    assert configured.normal_force.torque_feedback_gain == 0.0
    assert configured.normal_force.stiffness is not None
    # 刚度估计照常运行（trace 可比），位置前馈由 LADRC 取代，力矩前馈对标 full。
    assert configured.normal_force.stiffness.enabled is True
    assert configured.normal_force.stiffness.position_feedforward_gain == 0.0
    assert source.normal_force is not None
    assert source.normal_force.stiffness is not None
    assert (
        configured.normal_force.stiffness.torque_feedforward_gain
        == source.normal_force.stiffness.torque_feedforward_gain
    )
    # MIT kp/kd 不在 profile 层改动，接近阶段与各变体共享位置伺服。
    assert source.mit is not None
    assert configured.mit is not None
    assert configured.mit.kp == source.mit.kp
    assert configured.mit.kd == source.mit.kd
    # 源 profile 不携带 adrc 配置，默认行为保持不变。
    assert source.normal_force.adrc is None


def test_adrc_torque_variant_enables_model_scheduled_direct_torque_loop() -> None:
    """adrc-torque 注入二阶 LADRC 参数，并保留显式名义模型前馈。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    configured = configure_force_controller(source, variant="adrc-torque")

    assert configured.normal_force is not None
    assert configured.normal_force.torque_adrc == TorqueAdrcControl()
    assert configured.normal_force.torque_adrc.measurement_filter_cutoff_hz == 40.0
    assert configured.normal_force.adrc is None
    assert configured.normal_force.torque_feedback_gain == 0.0
    assert configured.normal_force.stiffness is not None
    assert configured.normal_force.stiffness.enabled is True
    assert configured.normal_force.stiffness.position_feedforward_gain == 0.0
    assert configured.normal_force.stiffness.torque_feedforward_gain == 1.0
    # 接近阶段仍复用相同 MIT 阻抗参数，旁路只发生在跟踪控制周期。
    assert configured.mit == source.mit


def test_adrc_torque_variant_accepts_explicit_tuning_override() -> None:
    """调参 study 可注入二阶直接力矩 ADRC 参数，而不修改源 profile。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    override = TorqueAdrcControl(
        measurement_filter_cutoff_hz=60.0,
        controller_bandwidth_rad_s=40.0,
        observer_bandwidth_rad_s=120.0,
    )

    configured = configure_force_controller(
        source,
        variant="adrc-torque",
        torque_adrc_override=override,
    )

    assert configured.normal_force is not None
    assert configured.normal_force.torque_adrc == override
    with pytest.raises(ValueError, match="requires a torque ADRC"):
        configure_force_controller(source, variant="full", torque_adrc_override=override)


def test_adrc_torque_td_variant_only_adds_reference_shaping() -> None:
    """TD 工程变体与论文式直接力矩 ADRC 仅参考整形配置不同。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    plain = configure_force_controller(source, variant="adrc-torque")
    shaped = configure_force_controller(source, variant="adrc-torque-td")

    assert plain.normal_force is not None
    assert shaped.normal_force is not None
    assert plain.normal_force.torque_adrc is not None
    assert shaped.normal_force.torque_adrc is not None
    assert plain.normal_force.torque_adrc.tracking_differentiator_bandwidth_rad_s is None
    assert shaped.normal_force.torque_adrc.tracking_differentiator_bandwidth_rad_s == 180.0
    assert (
        shaped.normal_force.torque_adrc.model_copy(
            update={"tracking_differentiator_bandwidth_rad_s": None}
        )
        == plain.normal_force.torque_adrc
    )


def test_force_reference_samples_derivatives_for_linear_and_smoothstep() -> None:
    """目标曲线同时提供二阶力矩 ADRC 所需的一、二阶参考导数。"""
    waypoints = (ForceWaypoint(t_s=0.0, force_n=2.0), ForceWaypoint(t_s=2.0, force_n=6.0))

    assert ForceReference(interpolation="linear", waypoints=waypoints).sample_at(1.0) == (
        4.0,
        2.0,
        0.0,
    )
    force, rate, acceleration = ForceReference(
        interpolation="smoothstep", waypoints=waypoints
    ).sample_at(0.5)
    assert force == pytest.approx(2.625)
    assert rate == pytest.approx(2.25)
    assert acceleration == pytest.approx(3.0)


def test_controller_variant_rejects_unknown_name_and_negative_seed() -> None:
    """运行时消融覆盖必须通过名称和随机种子校验。"""
    profile = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")

    with pytest.raises(ValueError, match="controller_variant"):
        configure_force_controller(profile, variant="unknown")  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="non-negative"):
        configure_force_controller(profile, sensor_noise_seed=-1)
    with pytest.raises(ValueError, match="stiffness_estimator_method"):
        configure_force_controller(
            profile,
            stiffness_estimator_method="unknown",  # type: ignore[arg-type]
        )


def test_stiffness_estimator_method_is_a_runtime_profile_override() -> None:
    """估计器对比可覆盖方法而不修改源 profile。"""
    source = load_profile(ROOT / "configs/custom_parallel_gripper.yaml")

    configured = configure_force_controller(
        source,
        variant="pid-stiffness-ff",
        stiffness_estimator_method="window_quadratic",
    )

    assert configured.normal_force is not None
    assert configured.normal_force.stiffness is not None
    assert configured.normal_force.stiffness.method == "window_quadratic"
    assert configured.normal_force.stiffness.torque_feedforward_gain == 0.0
    assert source.normal_force is not None
    assert source.normal_force.stiffness is not None
    assert source.normal_force.stiffness.method == "window_linear"


def test_force_tracking_run_writes_dynamic_reference_trace(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, fast_png_render: None
) -> None:
    """短版动态力跟踪实验写出 trace，并计算非空跟踪指标。"""
    task = ForceTrackingTask(
        schema_version=1,
        name="short_force_track",
        reference=ForceReference(
            interpolation="linear",
            waypoints=(
                ForceWaypoint(t_s=0.0, force_n=2.0),
                ForceWaypoint(t_s=0.4, force_n=6.0),
                ForceWaypoint(t_s=0.8, force_n=4.0),
            ),
        ),
    )
    output_csv = tmp_path / "force_track.csv"
    output_parquet = tmp_path / "force_track.parquet"
    output_plot = tmp_path / "force_track.png"
    plotted_rows: list[dict[str, float | str]] = []

    def capture_plot(
        path: Path, rows: list[dict[str, float | str]], *, task: ForceTrackingTask
    ) -> None:
        """记录直接 API 的完整绘图输入，并继续生成输出工件。"""
        assert path == output_plot
        assert task.name == "short_force_track"
        plotted_rows.extend(rows)
        _plot_force_tracking(path, rows, task=task)

    monkeypatch.setattr(force_tracking_module, "_plot_force_tracking", capture_plot)

    result = run_force_tracking(
        ROOT / "configs/custom_parallel_gripper.yaml",
        task=task,
        output_csv=output_csv,
        output_parquet=output_parquet,
        output_plot=output_plot,
        trace_sample_period_s=0.01,
        trace_event_window_s=0.02,
    )

    assert result.passed
    assert result.rmse_n >= 0.0
    assert output_csv.is_file()
    assert output_plot.is_file()
    assert output_plot.with_suffix(".pdf").is_file()
    with output_csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    tracking_rows = [row for row in rows if row["phase"] == "track_reference"]
    assert tracking_rows
    assert len({round(float(row["target_normal_force_n"]), 3) for row in tracking_rows}) > 1
    assert float(tracking_rows[0]["force_feedforward_torque_n_m"]) > 0.0
    assert float(tracking_rows[0]["estimated_contact_stiffness_n_per_m"]) > 0.0
    parquet = pq.ParquetFile(output_parquet)
    assert parquet.metadata is not None
    assert parquet.metadata.row_group(0).column(0).compression == "ZSTD"
    schema = parquet.schema_arrow
    assert schema.metadata is not None
    assert schema.metadata[b"pgt.trace.schema_version"] == b"1"
    assert schema.metadata[b"pgt.trace.compression"] == b"zstd"
    assert schema.names == list(rows[0])
    assert schema.field("phase").type == "string"
    assert schema.field("multiccd_enabled").type == "bool"
    assert schema.field("stiffness_position_limited").type == "bool"
    assert schema.field("active_taxel_contacts").type == "int64"
    assert schema.field("time_s").type == "double"
    parquet_rows = parquet.read().to_pylist()
    assert parquet_rows
    assert isinstance(parquet_rows[0]["multiccd_enabled"], bool)
    assert any(math.isnan(row["torque_adrc_measurement_n"]) for row in parquet_rows)
    assert all(row["stiffness_position_limited"] is False for row in parquet_rows)
    assert len(parquet_rows) == len(rows) < len(plotted_rows)
    configured = configure_force_controller(
        load_profile(ROOT / "configs/custom_parallel_gripper.yaml")
    )
    assert configured.mit is not None
    expected_metrics = _evaluate_tracking(
        plotted_rows,
        ignore_initial_s=task.metrics.ignore_initial_s,
        mit_t_max=float(configured.mit.t_max),
        p_min=float(configured.mit.p_min),
        p_max=float(configured.mit.p_max),
    )
    assert result.rmse_n == pytest.approx(expected_metrics[0])
    assert result.stiffness_position_limit_ratio == 0.0
    assert schema.metadata[b"pgt.trace.sample_period_s"] == b"0.01"
    assert schema.metadata[b"pgt.trace.event_window_s"] == b"0.02"


def test_force_tracking_direct_torque_run_tracks_reference(tmp_path: Path) -> None:
    """direct-torque 短版运行完成跟踪：位置修正诊断为 0，前馈力矩仍写入命令。"""
    task = ForceTrackingTask(
        schema_version=1,
        name="short_direct_torque",
        reference=ForceReference(
            interpolation="linear",
            waypoints=(
                ForceWaypoint(t_s=0.0, force_n=2.0),
                ForceWaypoint(t_s=0.4, force_n=6.0),
                ForceWaypoint(t_s=0.8, force_n=4.0),
            ),
        ),
    )
    output_csv = tmp_path / "direct_torque.csv"

    result = run_force_tracking(
        ROOT / "configs/custom_parallel_gripper.yaml",
        task=task,
        controller_variant="direct-torque",
        output_csv=output_csv,
    )

    assert result.passed
    assert math.isfinite(result.rmse_n)
    assert output_csv.is_file()
    with output_csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    tracking_rows = [row for row in rows if row["phase"] == "track_reference"]
    assert tracking_rows
    # 直接力矩路径：PID 与刚度位置修正诊断字段为 0，刚度估计仍在运行。
    assert all(float(row["pid_position_adjustment_rad"]) == 0.0 for row in tracking_rows)
    assert all(float(row["stiffness_position_adjustment_rad"]) == 0.0 for row in tracking_rows)
    assert all(
        math.isfinite(float(row["estimated_contact_stiffness_n_per_m"])) for row in tracking_rows
    )
    assert any(float(row["force_feedforward_torque_n_m"]) != 0.0 for row in tracking_rows)


@pytest.mark.parametrize("controller_variant", ["adrc-torque", "adrc-torque-td"])
def test_force_tracking_adrc_torque_run_writes_observer_diagnostics(
    tmp_path: Path, controller_variant: str
) -> None:
    """直接力矩 ADRC 两种参考路径均稳定完成并写出完整诊断量。"""
    task = ForceTrackingTask(
        schema_version=1,
        name="short_adrc_torque",
        reference=ForceReference(
            interpolation="smoothstep",
            waypoints=(
                ForceWaypoint(t_s=0.0, force_n=1.0),
                ForceWaypoint(t_s=0.4, force_n=3.0),
                ForceWaypoint(t_s=0.8, force_n=2.0),
            ),
        ),
    )
    output_csv = tmp_path / f"{controller_variant}.csv"

    result = run_force_tracking(
        ROOT / "configs/custom_parallel_gripper.yaml",
        task=task,
        controller_variant=controller_variant,  # type: ignore[arg-type]
        output_csv=output_csv,
    )

    assert result.passed
    assert math.isfinite(result.rmse_n)
    with output_csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    tracking_rows = [row for row in rows if row["phase"] == "track_reference"]
    assert tracking_rows
    assert all(float(row["force_position_adjustment_rad"]) == 0.0 for row in tracking_rows)
    assert all(float(row["pid_position_adjustment_rad"]) == 0.0 for row in tracking_rows)
    assert all(float(row["stiffness_position_adjustment_rad"]) == 0.0 for row in tracking_rows)
    assert all(float(row["force_feedforward_torque_n_m"]) > 0.0 for row in tracking_rows)
    assert all(math.isfinite(float(row["torque_adrc_measurement_n"])) for row in tracking_rows)
    assert all(math.isfinite(float(row["torque_adrc_reference_force_n"])) for row in tracking_rows)
    assert all(math.isfinite(float(row["torque_adrc_estimated_force_n"])) for row in tracking_rows)
    assert all(
        math.isfinite(float(row["torque_adrc_input_gain_n_per_n_m_s2"])) for row in tracking_rows
    )


def test_force_tracking_on_frame_receives_monotonic_snapshots() -> None:
    """逐帧回调按仿真时间单调推进，且不改变默认运行的跟踪结果。"""
    task = ForceTrackingTask(
        schema_version=1,
        name="short_on_frame",
        reference=ForceReference(
            interpolation="linear",
            waypoints=(
                ForceWaypoint(t_s=0.0, force_n=2.0),
                ForceWaypoint(t_s=0.4, force_n=6.0),
                ForceWaypoint(t_s=0.8, force_n=4.0),
            ),
        ),
    )
    profile = ROOT / "configs/custom_parallel_gripper.yaml"
    baseline = run_force_tracking(profile, task=task)
    times: list[float] = []
    phases: list[str] = []

    def capture(row: dict[str, object], model: object, data: object) -> None:
        del model, data
        times.append(float(row["time_s"]))
        phases.append(str(row["phase"]))

    result = run_force_tracking(profile, task=task, render_fps=20.0, on_frame=capture)

    assert result.passed
    assert result.rmse_n == pytest.approx(baseline.rmse_n)
    assert result.mae_n == pytest.approx(baseline.mae_n)
    assert len(times) >= 5
    assert times == sorted(times)
    assert times[0] < times[-1]
    assert float(times[-1]) * 20.0 >= len(times) - 2
    assert any(phase == "track_reference" for phase in phases)
