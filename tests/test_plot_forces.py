"""验证力曲线 CSV 读取。"""

from __future__ import annotations

from pathlib import Path

from parallel_gripper_tactile import analysis as plotter


def test_read_force_csv_parses_recorder_columns(tmp_path: Path) -> None:
    """按记录器列结构解析力曲线 CSV。"""
    csv_path = tmp_path / "forces.csv"
    csv_path.write_text(
        "step,time_s,control,left_fx,left_fy,left_fz,right_fx,right_fy,right_fz\n"
        "0,0.002,10,1,2,3,4,5,6\n",
        encoding="utf-8",
    )
    assert plotter.read_force_csv(csv_path) == {
        "time_s": [0.002],
        "control": [10.0],
        "left_fx": [1.0],
        "left_fy": [2.0],
        "left_fz": [3.0],
        "right_fx": [4.0],
        "right_fy": [5.0],
        "right_fz": [6.0],
    }
