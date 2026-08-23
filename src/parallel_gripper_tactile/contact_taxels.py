"""Read three-axis tactile forces directly from named MuJoCo contact geoms."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .profiles import GripperProfile, TactileLayout


@dataclass(frozen=True, slots=True)
class ContactTaxelFrame:
    """Forces applied to the two tactile surfaces in their local site frames.

    Each array has shape ``(3, rows, cols)`` in ``Fx, Fy, Fz`` order.  The
    sign is the force applied *to* the Pillar, so a correctly oriented tactile
    site reports positive ``Fz`` under compression.
    """

    left: np.ndarray
    right: np.ndarray


@dataclass(frozen=True, slots=True)
class _TaxelChannel:
    """Compiled IDs and grid coordinates for one collision taxel."""

    side: str
    row: int
    column: int
    site_id: int


class ContactTaxelReader:
    """Aggregate MuJoCo contact forces for a ``contact_geom`` tactile layout."""

    def __init__(self, model: mujoco.MjModel, tactile: TactileLayout) -> None:
        """Resolve the profile's named collision geoms and corresponding sites."""
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
                    self._channels[geom_id] = _TaxelChannel(side, row, column, site_id)

    @classmethod
    def from_profile(cls, model: mujoco.MjModel, profile: GripperProfile) -> "ContactTaxelReader":
        """Construct a reader using the tactile naming convention in a profile."""
        return cls(model, profile.tactile)

    @staticmethod
    def _force_on_geom_world(contact, contact_force: np.ndarray, geom_id: int) -> np.ndarray:
        """Return the contact force applied to ``geom_id`` in world coordinates.

        ``mj_contactForce`` expresses force in the contact frame and reports the
        force on ``geom2``.  The first contact-frame row is its normal axis, so
        the transpose converts contact-frame vectors to world coordinates.
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
        """Return the current left and right ``(3, rows, cols)`` force grids."""
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
