"""双侧法向力的最小触觉接触判定工具。"""

from __future__ import annotations

import math


def within_zero_window(left_force_n: float, right_force_n: float, threshold_n: float) -> bool:
    """判断双侧法向力是否同时落在零力窗口内。

    Args:
        left_force_n: 左侧法向力 (N)，可为负数（bias 后的残余读数）。
        right_force_n: 右侧法向力 (N)。
        threshold_n: 零窗口阈值 (N)，必须为非负数。

    Returns:
        bool: 两侧绝对值均不超过阈值时为 True。

    Raises:
        ValueError: 输入非有限或阈值为负时抛出。
    """
    values = (left_force_n, right_force_n, threshold_n)
    if not all(math.isfinite(value) for value in values) or threshold_n < 0.0:
        raise ValueError("零力窗口参数无效")
    return abs(left_force_n) <= threshold_n and abs(right_force_n) <= threshold_n


class ContactDetector:
    """要求左右触觉力同时越过阈值并保持稳定时间。"""

    def __init__(self, threshold_n: float, stable_s: float) -> None:
        """创建双侧接触判定器。

        Args:
            threshold_n: 每侧接触力阈值 (N)。
            stable_s: 双侧同时越过阈值的持续时间 (s)。
        """
        self._threshold_n = threshold_n
        self._stable_s = stable_s
        self._started_s: float | None = None

    def reset(self) -> None:
        """清空已累计的稳定时间，使下一次接触重新计时。"""
        self._started_s = None

    def update(self, left_force_n: float, right_force_n: float, now_s: float) -> bool:
        """更新双侧力并返回稳定接触结果。

        Args:
            left_force_n: 左侧法向力 (N)。
            right_force_n: 右侧法向力 (N)。
            now_s: 单调时钟时间 (s)。

        Returns:
            bool: 已形成稳定双侧接触时为 True。
        """
        if min(left_force_n, right_force_n) < self._threshold_n:
            self._started_s = None
            return False
        if self._started_s is None:
            self._started_s = now_s
            return False
        return now_s - self._started_s >= self._stable_s
