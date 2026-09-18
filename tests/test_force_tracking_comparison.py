"""验证力跟踪控制器对比 study 的 schema 与矩阵展开。"""

from __future__ import annotations

from pathlib import Path

import pytest

from parallel_gripper_tactile.studies.force_tracking_comparison import (
    StudyConfigError,
    load_comparison_config,
)


ROOT = Path(__file__).resolve().parents[1]


def _write_config(tmp_path: Path, body: str) -> Path:
    """写入最小有效配置，并允许测试追加或覆盖字段。"""
    config_path = tmp_path / "comparison.yaml"
    config_path.write_text(body, encoding="utf-8")
    return config_path


def test_comparison_config_resolves_paths_and_expands_ordered_matrix(tmp_path: Path) -> None:
    """所有路径相对 YAML 解析，矩阵按四个维度的配置顺序展开。"""
    config = load_comparison_config(
        _write_config(
            tmp_path,
            """name: smoke
profile: profile.yaml
tasks: [tasks/step.yaml, tasks/ramp.yaml]
controllers: [pid-only, pid-torque-ff]
materials: [soft, hard]
seeds: {start: 2, count: 3}
output_root: results
""",
        )
    )

    assert config.profile == tmp_path / "profile.yaml"
    assert config.tasks == (tmp_path / "tasks/step.yaml", tmp_path / "tasks/ramp.yaml")
    assert config.output_root == tmp_path / "results"
    assert len(config.conditions()) == 24
    assert config.conditions()[0] == ("pid-only", tmp_path / "tasks/step.yaml", "soft", 2)
    assert config.conditions()[-1] == ("pid-torque-ff", tmp_path / "tasks/ramp.yaml", "hard", 4)


def test_default_comparison_uses_shifted_contact_presets() -> None:
    """默认正式矩阵排除旧 soft、位置限幅和一阶 ADRC，并锁定默认估计器。"""
    config = load_comparison_config(ROOT / "configs/research/force_controller_selection/study.yaml")

    assert config.materials == ("medium", "hard", "stiff")
    assert config.stiffness_estimator_method == "window_linear"
    assert "adrc" not in config.controllers
    assert "pid-stiffness-limit" not in config.controllers
    assert "pid-stiffness-rate" in config.controllers
    assert "adrc-torque" in config.controllers
    assert "pid-stiffness-ff" not in config.controllers
    assert "full" not in config.controllers
    assert len(config.conditions()) == 108


@pytest.mark.parametrize(
    "contents",
    [
        "tasks: []",
        "tasks: [step.yaml, step.yaml]",
        "controllers: []",
        "controllers: [pid-only, pid-only]",
        "materials: []",
        "materials: [hard, hard]",
    ],
)
def test_comparison_config_rejects_empty_or_duplicate_dimensions(
    tmp_path: Path, contents: str
) -> None:
    """每个条件维度都必须非空且不包含重复值。"""
    config_path = _write_config(
        tmp_path,
        """profile: profile.yaml
tasks: [step.yaml]
controllers: [pid-only]
materials: [hard]
"""
        + contents
        + "\n",
    )
    with pytest.raises(StudyConfigError):
        load_comparison_config(config_path)


@pytest.mark.parametrize(
    "contents",
    [
        "controllers: [unknown]",
        "materials: [rubber]",
        "seeds: {start: -1, count: 1}",
        "seeds: {start: 0, count: 0}",
        "unexpected: true",
    ],
)
def test_comparison_config_rejects_invalid_values_and_unknown_fields(
    tmp_path: Path, contents: str
) -> None:
    """非法控制器、材料、种子范围和未知字段均在加载时失败。"""
    config_path = _write_config(
        tmp_path,
        """profile: profile.yaml
tasks: [step.yaml]
controllers: [pid-only]
materials: [hard]
"""
        + contents
        + "\n",
    )
    with pytest.raises(StudyConfigError):
        load_comparison_config(config_path)
