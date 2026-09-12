"""倒水实验的离线可校验配置。"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any, Literal

import yaml
from dmgripper_hardware import DEFAULT_USB2CAN_PORT
from papillarray_hardware import DEFAULT_PAPILLARRAY_PORT

from .config import ForceDemoConfig


def _finite_number(value: object, name: str, *, positive: bool = False) -> float:
    """验证一个配置数值。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限数值")
    number = float(value)
    if positive and number <= 0.0:
        raise ValueError(f"{name} 必须大于 0")
    return number


@dataclass(frozen=True, slots=True)
class PIDConfig:
    """PID 力跟踪器参数。"""

    kp: float = 0.016
    ki: float = 0.2
    kd: float = 0.0
    max_position_adjustment_rad: float = 0.15

    def __post_init__(self) -> None:
        """验证 PID 参数。"""
        for name in ("kp", "ki", "kd"):
            if _finite_number(getattr(self, name), name) < 0.0:
                raise ValueError(f"{name} 不得为负")
        _finite_number(
            self.max_position_adjustment_rad, "max_position_adjustment_rad", positive=True
        )


@dataclass(frozen=True, slots=True)
class ADRCConfig:
    """ADRC 力跟踪器参数。"""

    b0_n_per_m: float = 1000.0
    controller_bandwidth_rad_s: float = 5.0
    observer_bandwidth_rad_s: float = 20.0
    max_closing_velocity_m_s: float = 0.002

    def __post_init__(self) -> None:
        """验证 ADRC 参数。"""
        for name in (item.name for item in fields(self)):
            _finite_number(getattr(self, name), name, positive=True)


@dataclass(frozen=True, slots=True)
class GripConfig:
    """倒水期间的抓握安全参数。"""

    max_target_force_n: float = 1.5
    shear_gain: float = 1.0
    shear_threshold_n: float = 0.06
    max_force_rate_n_s: float = 0.5
    filter_tau_s: float = 0.05
    stable_time_s: float = 2.0
    force_tolerance_n: float = 0.15
    force_deadband_n: float = 0.1
    prevent_unloading: bool = True

    def __post_init__(self) -> None:
        """验证抓握参数。"""
        for name in (
            "max_target_force_n",
            "shear_gain",
            "max_force_rate_n_s",
            "filter_tau_s",
            "stable_time_s",
        ):
            _finite_number(getattr(self, name), name, positive=True)
        _finite_number(self.shear_threshold_n, "shear_threshold_n", positive=True)
        if _finite_number(self.force_tolerance_n, "force_tolerance_n") < 0.0:
            raise ValueError("force_tolerance_n 不得为负")
        if _finite_number(self.force_deadband_n, "force_deadband_n") < 0.0:
            raise ValueError("force_deadband_n 不得为负")
        if not isinstance(self.prevent_unloading, bool):
            raise ValueError("prevent_unloading 必须是布尔值")


@dataclass(frozen=True, slots=True)
class CupConfig:
    """DMgripper 真机倒水实验配置。"""

    controller: Literal["admittance", "pid", "adrc"] = "admittance"
    control: ForceDemoConfig = field(default_factory=ForceDemoConfig)
    dm_port: str = DEFAULT_USB2CAN_PORT
    tactile_port: str = DEFAULT_PAPILLARRAY_PORT
    max_control_gap_s: float = 0.1
    pid: PIDConfig = field(default_factory=PIDConfig)
    adrc: ADRCConfig = field(default_factory=ADRCConfig)
    grip: GripConfig = field(default_factory=GripConfig)

    def __post_init__(self) -> None:
        """验证控制器选择、端口和跨字段安全约束。"""
        if self.controller not in {"admittance", "pid", "adrc"}:
            raise ValueError("controller 必须是 admittance、pid 或 adrc")
        if not isinstance(self.control, ForceDemoConfig):
            raise ValueError("control 必须是 ForceDemoConfig")
        for item in fields(self.control):
            _finite_number(getattr(self.control, item.name), f"control.{item.name}")
        for name in (
            "zero_force_stable_s",
            "approach_endpoint_hold_s",
            "approach_feedforward_force_n",
        ):
            if getattr(self.control, name) < 0.0:
                raise ValueError(f"control.{name} 不得为负")
        if not 0.0 <= self.control.approach_feedforward_ratio <= 1.0:
            raise ValueError("control.approach_feedforward_ratio 必须位于 0 与 1 之间")
        if not isinstance(self.pid, PIDConfig):
            raise ValueError("pid 必须是 PIDConfig")
        if not isinstance(self.adrc, ADRCConfig):
            raise ValueError("adrc 必须是 ADRCConfig")
        if not isinstance(self.grip, GripConfig):
            raise ValueError("grip 必须是 GripConfig")
        control_gap_s = _finite_number(self.max_control_gap_s, "max_control_gap_s", positive=True)
        if control_gap_s <= 1.0 / self.control.control_rate_hz:
            raise ValueError("max_control_gap_s 必须大于一个控制周期")
        if control_gap_s > self.control.tactile_timeout_s:
            raise ValueError("max_control_gap_s 不得大于 control.tactile_timeout_s")
        for name in ("dm_port", "tactile_port"):
            if not isinstance(getattr(self, name), str) or not getattr(self, name).strip():
                raise ValueError(f"{name} 必须是非空字符串")
        if not self.grip.max_target_force_n >= self.control.target_force_n:
            raise ValueError("max_target_force_n 必须不小于 control.target_force_n")
        if not self.grip.max_target_force_n < self.control.force_ceiling_n:
            raise ValueError("max_target_force_n 必须严格小于 control.force_ceiling_n")
        if not self.control.target_force_n >= self.control.contact_on_n:
            raise ValueError("control.target_force_n 必须不小于 control.contact_on_n")
        if not self.control.zero_force_threshold_n < self.control.contact_on_n:
            raise ValueError("control.zero_force_threshold_n 必须小于 control.contact_on_n")
        if self.grip.force_deadband_n > self.grip.force_tolerance_n:
            raise ValueError("grip.force_deadband_n 不得大于 grip.force_tolerance_n")


_SCHEMA: dict[str, object] = {
    "controller": str,
    "control": ForceDemoConfig,
    "dm_port": str,
    "tactile_port": str,
    "max_control_gap_s": float,
    "pid": PIDConfig,
    "adrc": ADRCConfig,
    "grip": GripConfig,
}


def _strict_mapping(value: object, name: str) -> dict[str, object]:
    """取得字符串键的 YAML 映射。"""
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{name} 必须是字符串键的映射")
    return value


def _decode_dataclass(data: object, cls: type[Any], name: str) -> object:
    """按 dataclass 字段严格解码 YAML 映射。"""
    mapping = _strict_mapping(data, name)
    permitted = {item.name for item in fields(cls)}
    unknown = set(mapping) - permitted
    if unknown:
        raise ValueError(f"{name} 包含未知字段：{', '.join(sorted(unknown))}")
    kwargs: dict[str, object] = {}
    for item in fields(cls):
        if item.name not in mapping:
            continue
        value = mapping[item.name]
        # 通过默认值识别布尔开关，避免依赖 postponed annotations 的运行时表示。
        if isinstance(item.default, bool):
            if not isinstance(value, bool):
                raise ValueError(f"{name}.{item.name} 必须是布尔值")
            kwargs[item.name] = value
        else:
            kwargs[item.name] = _finite_number(value, f"{name}.{item.name}")
    return cls(**kwargs)


def load_cup_config(path: Path) -> CupConfig:
    """从严格 YAML 文件加载倒水配置。"""
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"无法读取配置文件：{path}") from error
    except yaml.YAMLError as error:
        raise ValueError(f"YAML 格式无效：{path}") from error
    mapping = _strict_mapping(data, "配置根节点")
    unknown = set(mapping) - set(_SCHEMA)
    if unknown:
        raise ValueError(f"配置根节点包含未知字段：{', '.join(sorted(unknown))}")
    kwargs: dict[str, object] = {}
    for name, cls in _SCHEMA.items():
        if name not in mapping:
            continue
        value = mapping[name]
        if name == "controller":
            if not isinstance(value, str):
                raise ValueError("controller 必须是字符串")
            kwargs[name] = value
        elif name in {"dm_port", "tactile_port"}:
            if not isinstance(value, str):
                raise ValueError(f"{name} 必须是字符串")
            kwargs[name] = value
        elif name == "max_control_gap_s":
            kwargs[name] = _finite_number(value, name)
        else:
            kwargs[name] = _decode_dataclass(value, cls, name)
    return CupConfig(**kwargs)


def cup_config_record(config: CupConfig) -> dict[str, object]:
    """返回可 JSON 序列化的普通配置记录。"""
    return asdict(config)
