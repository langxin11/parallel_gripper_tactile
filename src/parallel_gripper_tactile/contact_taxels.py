"""直接从具名 MuJoCo 接触几何体读取三轴触觉力。"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .profiles import GripperProfile, TactileLayout


@dataclass(frozen=True, slots=True)
class ContactTaxelFrame:
    """作用于两个触觉表面、位于其局部 site 坐标系中的力。

    每个数组的形状为 ``(3, rows, cols)``，顺序为 ``Fx, Fy, Fz``。
    符号表示施加到 Pillar *上* 的力，因此方向正确的触觉 site
    在受压时报告正的 ``Fz``。
    """

    left: np.ndarray
    right: np.ndarray


@dataclass(frozen=True, slots=True)
class _TaxelChannel:
    """单个碰撞 taxel 的编译后 ID 与网格坐标。"""

    side: str
    row: int
    column: int
    site_id: int
    geom_id: int


class ContactTaxelReader:
    """为 ``contact_geom`` 触觉布局聚合 MuJoCo 接触力。"""

    def __init__(self, model: mujoco.MjModel, tactile: TactileLayout) -> None:
        """解析 profile 中具名的碰撞几何体及其对应的 site。"""
        if tactile.mode != "contact_geom":
            raise ValueError(f"ContactTaxelReader requires contact_geom mode, got {tactile.mode!r}")
        self._model = model
        self._rows = tactile.rows
        self._columns = tactile.cols
        self._channels: dict[int, _TaxelChannel] = {}
        for side in ("left", "right"):
            for row in range(tactile.rows):
                for column in range(tactile.cols):
                    geom_name = tactile.names(side)[row * tactile.cols + column]
                    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
                    if geom_id < 0:
                        raise ValueError(f"model is missing tactile geom {geom_name!r}")
                    site_name = geom_name.replace("_geom_", "_", 1)
                    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
                    if site_id < 0:
                        raise ValueError(
                            f"model is missing tactile site {site_name!r} for geom {geom_name!r}"
                        )
                    self._channels[geom_id] = _TaxelChannel(side, row, column, site_id, geom_id)

    @classmethod
    def from_profile(cls, model: mujoco.MjModel, profile: GripperProfile) -> "ContactTaxelReader":
        """使用 profile 中的触觉命名约定构造读取器。"""
        return cls(model, profile.tactile)

    @staticmethod
    def _force_on_geom_world(contact, contact_force: np.ndarray, geom_id: int) -> np.ndarray:
        """返回世界坐标系中施加到 ``geom_id`` 的接触力。

        ``mj_contactForce`` 在接触坐标系中表示力，并报告施加在 ``geom2``
        上的力。接触坐标系的第一行是其法向轴，因此转置可将接触坐标系向量
        转换为世界坐标。
        """
        force_on_geom2_world = (
            np.asarray(contact.frame, dtype=np.float64).reshape(3, 3).T @ contact_force[:3]
        )
        if geom_id == contact.geom2:
            return force_on_geom2_world
        if geom_id == contact.geom1:
            return -force_on_geom2_world
        raise ValueError(f"geom {geom_id} does not belong to this contact")

    def read(self, data: mujoco.MjData) -> ContactTaxelFrame:
        """返回当前的左右 ``(3, rows, cols)`` 力网格。"""
        forces = {
            "left": np.zeros((3, self._rows, self._columns), dtype=np.float64),
            "right": np.zeros((3, self._rows, self._columns), dtype=np.float64),
        }
        contact_force = np.empty(6, dtype=np.float64)
        for contact_id in range(data.ncon):
            contact = data.contact[contact_id]
            channels = [
                (geom_id, self._channels[geom_id])
                for geom_id in (contact.geom1, contact.geom2)
                if geom_id in self._channels
            ]
            if not channels:
                continue
            mujoco.mj_contactForce(self._model, data, contact_id, contact_force)
            for geom_id, channel in channels:
                force_world = self._force_on_geom_world(contact, contact_force, geom_id)
                site_rotation = np.asarray(data.site_xmat[channel.site_id]).reshape(3, 3)
                forces[channel.side][:, channel.row, channel.column] += (
                    site_rotation.T @ force_world
                )
        return ContactTaxelFrame(left=forces["left"], right=forces["right"])
