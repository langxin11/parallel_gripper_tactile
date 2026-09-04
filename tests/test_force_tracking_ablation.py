"""验证力跟踪消融 study 的 schema、矩阵与统计。"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest

from parallel_gripper_tactile.studies.force_tracking_ablation import (
    StudyConfigError,
    load_study_config,
)


ROOT = Path(__file__).resolve().parents[1]


def _protocol_module() -> object:
    path = Path(__file__).parents[1] / "scripts" / "experiments" / "force_tracking_ablation.py"
    spec = importlib.util.spec_from_file_location("force_tracking_ablation_protocol", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _row(*, seed: int, rmse: float, stiffness: float) -> dict[str, object]:
    return {
        "controller_variant": "pid-only",
        "object_material": "soft",
        "passed": True,
        "sensor_noise_seed": seed,
        "contact_time_s": 1.0,
        "tracking_start_time_s": 1.2,
        "rmse_n": rmse,
        "mae_n": 0.2,
        "peak_abs_error_n": 0.5,
        "mean_error_n": 0.1,
        "final_error_n": 0.05,
        "torque_saturation_ratio": 0.0,
        "position_saturation_ratio": 0.0,
        "mean_estimated_stiffness_n_per_m": stiffness,
    }


def test_study_config_loads_relative_paths_and_expands_matrix(tmp_path: Path) -> None:
    """路径相对于 YAML 定位，矩阵按三个维度完整展开。"""
    config_path = tmp_path / "study.yaml"
    config_path.write_text(
        """name: smoke
profile: profile.yaml
task: task.yaml
controllers: [pid-only, full]
materials: [soft, medium, hard]
seeds: {start: 2, count: 4}
output_root: results
""",
        encoding="utf-8",
    )
    config = load_study_config(config_path)
    assert config.profile == tmp_path / "profile.yaml"
    assert config.output_root == tmp_path / "results"
    assert len(config.conditions()) == 24
    assert config.conditions()[0] == ("pid-only", "soft", 2)


def test_default_ablation_uses_shifted_contact_presets() -> None:
    """默认消融矩阵排除旧 soft，并加入 stiff。"""
    config = load_study_config(ROOT / "configs/studies/force_tracking_ablation.yaml")

    assert config.materials == ("medium", "hard", "stiff")


@pytest.mark.parametrize(
    "contents",
    [
        "controllers: [unknown]",
        "materials: [rubber]",
        "seeds: {count: 0}",
        "controllers: []",
        "materials: []",
        "unexpected: true",
    ],
)
def test_study_config_rejects_invalid_conditions(tmp_path: Path, contents: str) -> None:
    """非法枚举、空矩阵和未知字段均在加载时失败。"""
    config_path = tmp_path / "study.yaml"
    config_path.write_text(
        "name: smoke\nprofile: profile.yaml\ntask: task.yaml\ncontrollers: [full]\nmaterials: [hard]\n"
        + contents
        + "\n",
        encoding="utf-8",
    )
    with pytest.raises(StudyConfigError):
        load_study_config(config_path)


def test_aggregation_skips_nonfinite_values_and_uses_json_null() -> None:
    """聚合跳过非有限数，并为 JSON 规范化其原始行。"""
    protocol = _protocol_module()
    rows = [_row(seed=1, rmse=0.2, stiffness=math.nan), _row(seed=2, rmse=0.4, stiffness=math.inf)]
    aggregate = protocol.aggregate_rows(rows)[0]  # type: ignore[attr-defined]
    assert aggregate["runs"] == 2
    assert aggregate["passed_runs"] == 2
    assert aggregate["rmse_n_mean"] == pytest.approx(0.3)
    assert aggregate["rmse_n_std"] == pytest.approx(2**0.5 * 0.1)
    assert aggregate["mean_estimated_stiffness_n_per_m_mean"] is None
    compatible = protocol.json_compatible({"runs": rows})  # type: ignore[attr-defined]
    assert compatible["runs"][0]["mean_estimated_stiffness_n_per_m"] is None  # type: ignore[index]


def test_single_sample_aggregate_has_no_sample_standard_deviation() -> None:
    """一个样本没有可定义的样本标准差。"""
    protocol = _protocol_module()
    aggregate = protocol.aggregate_rows([_row(seed=1, rmse=0.2, stiffness=10.0)])[0]  # type: ignore[attr-defined]
    assert aggregate["rmse_n_std"] is None


def test_ablation_figures_render_paired_factorial_effects(tmp_path: Path) -> None:
    """小型合成数据可生成材料总览和按相同 seed 配对的 PID 效应图。"""
    protocol = _protocol_module()
    rows = []
    for controller, rmse in (
        ("pid-only", 0.50),
        ("pid-torque-ff", 0.40),
        ("pid-stiffness-ff", 0.45),
        ("full", 0.32),
    ):
        row = _row(seed=3, rmse=rmse, stiffness=100.0)
        row.update(
            {
                "controller_variant": controller,
                "mae_n": rmse / 2.0,
                "torque_saturation_ratio": 0.01,
            }
        )
        rows.append(row)

    effects = protocol.paired_pid_factorial_effects(rows)  # type: ignore[attr-defined]
    assert effects["torque_effect"] == pytest.approx((-0.10, -0.13))
    figures = protocol.render_study_figures(  # type: ignore[attr-defined]
        rows,
        protocol.aggregate_rows(rows),  # type: ignore[attr-defined]
        tmp_path,
        controller_order=("pid-only", "pid-torque-ff", "pid-stiffness-ff", "full"),
    )

    assert [path.suffix for path in figures] == [".png", ".pdf", ".png", ".pdf"]
    assert all(path.is_file() and path.stat().st_size > 0 for path in figures)


def test_factorial_effects_ignore_incomplete_material_seed_blocks() -> None:
    """缺少任一 PID 变体的材料与 seed 组合不得进入 2×2 配对统计。"""
    protocol = _protocol_module()
    rows = []
    for controller, rmse in (
        ("pid-only", 0.50),
        ("pid-torque-ff", 0.40),
        ("pid-stiffness-ff", 0.45),
        ("full", 0.32),
    ):
        row = _row(seed=3, rmse=rmse, stiffness=100.0)
        row["controller_variant"] = controller
        rows.append(row)
    incomplete = _row(seed=4, rmse=9.0, stiffness=100.0)
    incomplete["controller_variant"] = "pid-only"
    rows.append(incomplete)

    effects = protocol.paired_pid_factorial_effects(rows)  # type: ignore[attr-defined]

    assert effects["none"] == [0.50]
    assert effects["torque_effect"] == pytest.approx((-0.10, -0.13))
