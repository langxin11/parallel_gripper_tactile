"""仿真步骤记录钩子。"""

from __future__ import annotations

import csv
from collections.abc import Sequence
from pathlib import Path


class ForceCsvRecorder:
    """将控制量及左右触觉表面受力记录为可直接绘图的 CSV。

    两个演示都使用“物体施加给触觉表面”的公共方向，压缩时局部 Fz 为正。
    taxel 演示在调用记录器前对原始 force sensor 向量整体取反；touch_grid
    记录各格点表面力之和。二者均在各自 site 局部坐标系中表达。
    """

    fieldnames = (
        "step",
        "time_s",
        "control",
        "left_fx",
        "left_fy",
        "left_fz",
        "right_fx",
        "right_fy",
        "right_fz",
    )

    def __init__(self, path: Path | None, every: int = 1) -> None:
        if every <= 0:
            raise ValueError("记录间隔必须为正整数。")
        self._every = every
        self._file = None
        self._writer = None
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._file = path.open("w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._file, fieldnames=self.fieldnames)
            self._writer.writeheader()

    def record(
        self,
        step: int,
        time_s: float,
        control: float,
        left_force: Sequence[float],
        right_force: Sequence[float],
    ) -> None:
        """记录一次物理步；此方法可作为仿真循环中的记录钩子调用。"""
        if self._writer is None or step % self._every:
            return
        self._writer.writerow(
            {
                "step": step,
                "time_s": time_s,
                "control": control,
                "left_fx": left_force[0],
                "left_fy": left_force[1],
                "left_fz": left_force[2],
                "right_fx": right_force[0],
                "right_fy": right_force[1],
                "right_fz": right_force[2],
            }
        )

    def close(self) -> None:
        """关闭记录文件。"""
        if self._file is not None:
            self._file.close()
            self._file = None
