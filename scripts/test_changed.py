"""按改动模块选择开发阶段的相关 pytest。"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FULL_SUITE_FILES = {
    ".pre-commit-config.yaml",
    "conftest.py",
    "pyproject.toml",
    "uv.lock",
}
NON_CODE_PREFIXES = ("docs/", "reports/", "CHANGELOG.md", "AGENTS.md", "CONTRIBUTING.md")
RELATED_PREFIXES = {
    "src/parallel_gripper_tactile/cli/": (
        "tests/test_cli.py",
        "tests/test_config_catalog.py",
    ),
    # 共享核改动的消费者映射：核心自身、真机实验包与仿真力跟踪回归。
    "packages/dm_grasp_core/": (
        "packages/dm_grasp_core/tests",
        "packages/dmgripper_experiments/tests",
        "tests/test_force_tracking.py",
    ),
    # 真机实验包改动的消费者映射：本包测试与共享核回归（跟踪接口契约）。
    "packages/dmgripper_experiments/": (
        "packages/dmgripper_experiments/tests",
        "packages/dm_grasp_core/tests",
    ),
    "packages/dmgripper_hardware/": ("packages/dmgripper_experiments/tests",),
    "packages/papillarray_hardware/": ("packages/dmgripper_experiments/tests",),
    "src/parallel_gripper_tactile/research/catalog.py": ("tests/test_config_catalog.py",),
    "configs/": (
        "tests/test_research_configuration.py",
        "tests/test_research_execution.py",
        "tests/test_research_study.py",
        "tests/test_study_lifecycle.py",
        "tests/test_study_progress.py",
    ),
    "scripts/research/": (
        "tests/test_research_configuration.py",
        "tests/test_research_execution.py",
        "tests/test_research_study.py",
        "tests/test_study_lifecycle.py",
        "tests/test_study_progress.py",
    ),
    "src/parallel_gripper_tactile/research/": (
        "tests/test_research_configuration.py",
        "tests/test_research_execution.py",
        "tests/test_research_study.py",
        "tests/test_study_lifecycle.py",
        "tests/test_study_progress.py",
    ),
    "src/parallel_gripper_tactile/studies/lifecycle.py": (
        "tests/test_study_lifecycle.py",
        "tests/test_study_progress.py",
        "tests/test_research_study.py",
        "tests/test_force_tracking_controller_comparison_script.py",
        "tests/test_friction_estimator_validation_study.py",
    ),
    "src/parallel_gripper_tactile/studies/protocols/": (
        "tests/test_study_lifecycle.py",
        "tests/test_study_progress.py",
        "tests/test_research_study.py",
        "tests/test_study_aggregation.py",
        "tests/test_force_tracking_controller_comparison_script.py",
    ),
    "src/parallel_gripper_tactile/control/": (
        "tests/test_control.py",
        "tests/test_force_tracking.py",
        "tests/test_force_scheduling.py",
    ),
    "src/parallel_gripper_tactile/perception/": (
        "tests/test_contact_taxels.py",
        "tests/test_friction_estimation.py",
        "tests/test_tactile_readers.py",
        "tests/test_tactile_slip.py",
        "tests/test_taxel_friction.py",
    ),
    "src/parallel_gripper_tactile/scenes/": (
        "tests/test_simulation.py",
        "tests/test_contact_taxels.py",
        "tests/test_custom_grasp_validation.py",
        "tests/test_compare_tactile_models.py",
    ),
    "src/parallel_gripper_tactile/simulation/": (
        "tests/test_simulation.py",
        "tests/test_timing.py",
        "tests/test_force_tracking.py",
        "tests/test_friction_estimation_experiment.py",
    ),
    "src/parallel_gripper_tactile/visualization/": (
        "tests/test_plotstyle.py",
        "tests/test_force_tracking.py",
        "tests/test_force_tracking_controller_comparison_script.py",
    ),
}


def _changed_paths(base: str | None) -> list[str]:
    """读取相对基线和工作区的已跟踪、未跟踪改动。"""
    revision = f"{base}...HEAD" if base else "HEAD"
    tracked = subprocess.run(
        ["git", "diff", "--name-only", "--diff-filter=ACMR", revision],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    if base:
        tracked.extend(
            subprocess.run(
                ["git", "diff", "--name-only", "--diff-filter=ACMR", "HEAD"],
                cwd=ROOT,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.splitlines()
        )
    untracked = subprocess.run(
        ["git", "ls-files", "--others", "--exclude-standard"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    return sorted(set(tracked + untracked))


def _matching_root_tests(stem: str) -> set[str]:
    """按实现模块名发现同名前缀测试。"""
    return {path.relative_to(ROOT).as_posix() for path in (ROOT / "tests").glob(f"test_{stem}*.py")}


def select_tests(paths: list[str]) -> tuple[str, ...] | None:
    """返回相关测试；``None`` 表示必须回退全量测试。"""
    selected: set[str] = set()
    for raw_path in paths:
        path = Path(raw_path).as_posix().removeprefix("./")
        if path in FULL_SUITE_FILES or path.startswith(".github/workflows/"):
            return None
        if path.startswith("tests/") and path.endswith(".py"):
            selected.add(path)
            continue
        if path.startswith("packages/"):
            parts = Path(path).parts
            if len(parts) >= 2:
                selected.add(f"packages/{parts[1]}/tests")
        matched = False
        for prefix, tests in RELATED_PREFIXES.items():
            if path.startswith(prefix):
                selected.update(tests)
                matched = True
        if path.startswith("packages/"):
            matched = True
        if not matched and path.startswith(("src/", "scripts/")) and path.endswith(".py"):
            same_name_tests = _matching_root_tests(Path(path).stem)
            selected.update(same_name_tests)
            matched = matched or bool(same_name_tests)
        if matched or path.startswith(NON_CODE_PREFIXES):
            continue
        if path.startswith(("src/", "scripts/", "configs/", "assets/")):
            return None
    return tuple(sorted(test for test in selected if (ROOT / test).exists()))


def main() -> int:
    """解析改动、显示选择结果并运行相关测试。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", help="显式指定相对仓库根目录的改动路径。")
    parser.add_argument("--base", help="纳入相对该 Git 基线的已提交改动，例如 origin/main。")
    parser.add_argument("--list", action="store_true", help="只显示将运行的测试。")
    args = parser.parse_args()

    paths = args.paths or _changed_paths(args.base)
    tests = select_tests(paths)
    if tests is None:
        tests = ()
        print("存在基础设施或未映射代码改动，保守回退到全量测试。")
    elif not tests:
        print("没有需要运行 pytest 的代码改动。")
        return 0
    else:
        print("相关测试：")
        for test in tests:
            print(f"  {test}")

    if args.list:
        return 0
    return subprocess.run(
        [sys.executable, "-m", "pytest", *tests], cwd=ROOT, check=False
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
