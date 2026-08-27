"""通用的触觉读取器接口以及基于 MuJoCo 的读取器实现。

此模块中的读取器刻意只依赖一个很小的结构化布局约定。因此，只要配置
模型与场景构建器提供 mode、网格维度以及 left/right 名称，它们就可以
独立演化。
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal, Protocol, runtime_checkable

import mujoco
import numpy as np
from numpy.typing import NDArray

from .contact_taxels import ContactTaxelReader

TactileMode = Literal["force_sensor", "contact_geom", "touch_grid"]
NameResolver = Callable[[str], str]


@dataclass(frozen=True, slots=True)
class TactileFrame:
    """两个触觉表面的局部三轴力。

    ``left`` 与 ``right`` 使用 ``(3, rows, cols)`` 顺序：``Fx, Fy, Fz``。
    力施加在触觉表面 *上*，因此沿局部 ``Fz`` 的压缩为正值。数组在构造时
    会被复制，这样调用方就无法通过输入缓冲区意外修改此前发出的采样。
    """

    left: NDArray[np.float64]
    right: NDArray[np.float64]

    def __post_init__(self) -> None:
        """规范化力网格并强制共享的读取器约定。"""
        left = _validated_grid(self.left, "left")
        right = _validated_grid(self.right, "right")
        if left.shape != right.shape:
            raise ValueError("left and right tactile grids must have the same shape")
        object.__setattr__(self, "left", left)
        object.__setattr__(self, "right", right)


@runtime_checkable
class TactileReader(Protocol):
    """从单个 MuJoCo 状态读取当前局部触觉力网格。"""

    @property
    def shape(self) -> tuple[int, int, int]:
        """返回常见的 ``(3, rows, cols)`` 输出形状。"""

    def read(self, data: mujoco.MjData) -> TactileFrame:
        """从 ``data`` 读取一个不可变的触觉采样。"""


@runtime_checkable
class TactileLayoutLike(Protocol):
    """由 :func:`create_tactile_reader` 所需的最小配置表面。"""

    mode: str
    rows: int
    cols: int


def _validated_grid(values: object, side: str) -> NDArray[np.float64]:
    """返回一个私有的、有限的 ``(3, rows, cols)`` 网格副本。"""
    grid = np.array(values, dtype=np.float64, copy=True)
    if grid.ndim != 3 or grid.shape[0] != 3 or grid.shape[1] <= 0 or grid.shape[2] <= 0:
        raise ValueError(f"{side} tactile grid must have shape (3, rows, cols)")
    if not np.all(np.isfinite(grid)):
        raise ValueError(f"{side} tactile grid must contain only finite values")
    grid.setflags(write=False)
    return grid


def _validate_layout(tactile: TactileLayoutLike) -> tuple[int, int]:
    """校验所有读取器实现共享的布局字段。"""
    if tactile.rows <= 0 or tactile.cols <= 0:
        raise ValueError("tactile rows and cols must be positive")
    return tactile.rows, tactile.cols


def _name_for(tactile: TactileLayoutLike, side: str, row: int, column: int) -> str:
    """为触觉布局构建按行优先排列的 taxel 传感器名称。"""
    prefix = getattr(tactile, f"{side}_prefix", "")
    if not prefix:
        raise ValueError(f"tactile {side}_prefix must be non-empty")
    return f"{prefix}{row}{column}"


def _object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    """解析具名的 MuJoCo 对象，否则抛出简明的约定错误。"""
    object_id = mujoco.mj_name2id(model, kind, name)
    if object_id < 0:
        raise ValueError(f"model is missing tactile {kind.name.lower()} {name!r}")
    return object_id


@dataclass(frozen=True, slots=True)
class _SensorChannel:
    """单个力 taxel 的传感器存储位置与输出位置。"""

    side: str
    row: int
    column: int
    address: int


class ForceSensorTactileReader:
    """读取盒式 taxel 力传感器并规范化其反作用力符号。"""

    def __init__(
        self,
        model: mujoco.MjModel,
        tactile: TactileLayoutLike,
        *,
        name_resolver: NameResolver | None = None,
    ) -> None:
        """解析两个触觉垫的、按行优先排列的三轴力传感器。"""
        if tactile.mode != "force_sensor":
            raise ValueError(
                f"ForceSensorTactileReader requires force_sensor mode, got {tactile.mode!r}"
            )
        self._rows, self._cols = _validate_layout(tactile)
        resolve = name_resolver or (lambda name: name)
        self._channels: list[_SensorChannel] = []
        for side in ("left", "right"):
            for row in range(self._rows):
                for column in range(self._cols):
                    name = resolve(_name_for(tactile, side, row, column))
                    sensor_id = _object_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
                    if model.sensor_dim[sensor_id] < 3:
                        raise ValueError(
                            f"tactile force sensor {name!r} must have at least 3 channels"
                        )
                    self._channels.append(
                        _SensorChannel(
                            side=side,
                            row=row,
                            column=column,
                            address=int(model.sensor_adr[sensor_id]),
                        )
                    )

    @property
    def shape(self) -> tuple[int, int, int]:
        """返回常见的力网格形状。"""
        return (3, self._rows, self._cols)

    def read(self, data: mujoco.MjData) -> TactileFrame:
        """返回局部力网格，并将压缩规范化为正的 Fz。"""
        forces = {
            "left": np.zeros(self.shape, dtype=np.float64),
            "right": np.zeros(self.shape, dtype=np.float64),
        }
        for channel in self._channels:
            # MuJoCo 的 site 力传感器报告 taxel -> pad 的反作用力；
            # 触觉约定则是作用在 taxel 上的表面载荷。
            forces[channel.side][:, channel.row, channel.column] = -data.sensordata[
                channel.address : channel.address + 3
            ]
        return TactileFrame(left=forces["left"], right=forces["right"])


class ContactGeomTactileReader:
    """将 :class:`ContactTaxelReader` 适配到常见触觉读取器协议。"""

    def __init__(
        self,
        model: mujoco.MjModel,
        tactile: TactileLayoutLike,
        *,
        name_resolver: NameResolver | None = None,
    ) -> None:
        """编译 contact-geom 通道，可选地解析场景前缀。"""
        if tactile.mode != "contact_geom":
            raise ValueError(
                f"ContactGeomTactileReader requires contact_geom mode, got {tactile.mode!r}"
            )
        self._rows, self._cols = _validate_layout(tactile)
        self._reader = _ResolvedContactTaxelReader(
            model, tactile, name_resolver or (lambda name: name)
        )

    @property
    def shape(self) -> tuple[int, int, int]:
        """返回常见的力网格形状。"""
        return (3, self._rows, self._cols)

    def read(self, data: mujoco.MjData) -> TactileFrame:
        """返回触觉 site 局部坐标系中的接触力。"""
        frame = self._reader.read(data)
        return TactileFrame(left=frame.left, right=frame.right)


class _ResolvedContactTaxelReader(ContactTaxelReader):
    """面向为所有夹爪对象添加命名空间的场景的接触读取器变体。"""

    def __init__(
        self, model: mujoco.MjModel, tactile: TactileLayoutLike, resolve: NameResolver
    ) -> None:
        """用一个完全解析后的几何体名称初始化临时布局。"""
        self._resolved_tactile = _ResolvedLayout(tactile, resolve)
        super().__init__(model, self._resolved_tactile)  # type: ignore[arg-type]


class _ResolvedLayout:
    """用于旧版 ``ContactTaxelReader`` 接口的小型兼容包装器。"""

    def __init__(self, tactile: TactileLayoutLike, resolve: NameResolver) -> None:
        self.mode = tactile.mode
        self.rows = tactile.rows
        self.cols = tactile.cols
        self._tactile = tactile
        self._resolve = resolve

    def names(self, side: str) -> tuple[str, ...]:
        """返回按行优先排列的、完全限定的碰撞几何体名称。"""
        return tuple(
            self._resolve(_name_for(self._tactile, side, row, column))
            for row in range(self.rows)
            for column in range(self.cols)
        )


class TouchGridTactileReader:
    """将 MuJoCo ``sensor.touch_grid`` 插件输出读取为局部 xyz 网格。"""

    def __init__(
        self,
        model: mujoco.MjModel,
        tactile: TactileLayoutLike,
        *,
        name_resolver: NameResolver | None = None,
    ) -> None:
        """为每个指尖解析一个三通道的 touch-grid 传感器。"""
        if tactile.mode != "touch_grid":
            raise ValueError(
                f"TouchGridTactileReader requires touch_grid mode, got {tactile.mode!r}"
            )
        self._rows, self._cols = _validate_layout(tactile)
        resolve = name_resolver or (lambda name: name)
        self._addresses: dict[str, int] = {}
        expected_dim = 3 * self._rows * self._cols
        for side in ("left", "right"):
            # touch-grid 配置将其完整传感器名称存储在使用前缀字段中。
            # 未来的 Pydantic profile 可能暴露显式的 ``left_sensor``/``right_sensor``
            # 字段；支持这些字段，而无需让此模块耦合到特定的配置类型。
            name = getattr(tactile, f"{side}_sensor", None) or (
                getattr(tactile, f"{side}_prefix", "")
            )
            name = resolve(name)
            sensor_id = _object_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if model.sensor_dim[sensor_id] != expected_dim:
                raise ValueError(
                    f"touch-grid sensor {name!r} has dimension {model.sensor_dim[sensor_id]}, "
                    f"expected {expected_dim} for a {self._rows}x{self._cols} grid"
                )
            self._addresses[side] = int(model.sensor_adr[sensor_id])

    @property
    def shape(self) -> tuple[int, int, int]:
        """返回常见的力网格形状。"""
        return (3, self._rows, self._cols)

    def read(self, data: mujoco.MjData) -> TactileFrame:
        """返回从插件 zxy 重排为触觉 xyz 的 touch-grid 输出。"""
        values = {
            side: data.sensordata[address : address + 3 * self._rows * self._cols].reshape(
                self.shape
            )[[1, 2, 0]]
            for side, address in self._addresses.items()
        }
        return TactileFrame(left=values["left"], right=values["right"])


def create_tactile_reader(
    model: mujoco.MjModel,
    tactile: TactileLayoutLike,
    *,
    name_resolver: NameResolver | None = None,
) -> TactileReader:
    """创建由已校验触觉布局 mode 选定的读取器。

    ``name_resolver`` 对在命名空间下挂接夹爪的场景构建器很有用
    （例如将 ``touch_left`` 变成 ``gripper/touch_left``）。
    """
    factories: dict[
        str, type[ForceSensorTactileReader | ContactGeomTactileReader | TouchGridTactileReader]
    ] = {
        "force_sensor": ForceSensorTactileReader,
        "contact_geom": ContactGeomTactileReader,
        "touch_grid": TouchGridTactileReader,
    }
    try:
        reader_type = factories[tactile.mode]
    except KeyError as error:
        raise ValueError(f"unsupported tactile mode {tactile.mode!r}") from error
    return reader_type(model, tactile, name_resolver=name_resolver)
