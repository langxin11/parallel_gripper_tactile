"""兼容导出 PapillArray 硬件包提供的通用采集会话。"""

from papillarray_hardware import TactileSnapshot, TactileWorker
from dataclasses import dataclass, fields
import math

from dm_grasp_core.tactile.multirate import TactilePreprocessor, TactileState


@dataclass(frozen=True, slots=True)
class MultirateSnapshot(TactileSnapshot):
    """同一包的原始与预处理状态，在采集线程内组装后一次发布。"""

    processed: TactileState | None = None


class HardwareTactilePreprocessor:
    """设备时间积分滤波，主机时间另存于原始快照；不改变控制权限。"""

    def __init__(self, config) -> None:
        """保存冻结配置；不访问硬件。"""
        adaptive = config.reference.adaptive
        self._processor = TactilePreprocessor(
            adaptive.tactile_sampling,
            load_tau_s=adaptive.unified.load.filter_tau_s,
            observer_config=adaptive.unified.observer,
        )
        self._ceiling = config.safety.force_ceiling_n
        self._sequence = -1

    def __call__(self, snapshot: TactileSnapshot) -> MultirateSnapshot:
        """先验证原始力保护，再预处理；异常由采集线程锁存并交运行时失能。"""
        if not all(math.isfinite(v) for v in (snapshot.raw_left_fz_n, snapshot.raw_right_fz_n)):
            raise ValueError("原始法向力非有限")
        if max(abs(snapshot.raw_left_fz_n), abs(snapshot.raw_right_fz_n)) > self._ceiling:
            raise ValueError("原始法向力超过保护上限")
        self._sequence += 1 + (snapshot.counter_gap or 0)
        state = self._processor.update(
            snapshot.left_taxel_forces_n,
            snapshot.right_taxel_forces_n,
            sample_time_s=snapshot.timestamp_us * 1e-6,
            sequence_id=self._sequence,
        )
        if not state.valid:
            raise ValueError("逐触点原始力无效或达到量程")
        return MultirateSnapshot(
            **{f.name: getattr(snapshot, f.name) for f in fields(TactileSnapshot)}, processed=state
        )


__all__ = ["TactileSnapshot", "TactileWorker"]
