"""Grasp-experiment timing protocols shared by demo, benchmark, and video scripts."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

Vector3 = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class DisturbanceProtocol:
    """Timing and waveform of the release-and-disturbance grasp experiment.

    The experiment closes the gripper, lets the object settle on a temporary
    support, removes the support, holds the object unsupported, applies a
    tangential sinusoidal force at the object's center of mass, and observes
    the recovery.  The force direction may rotate at constant angular velocity
    inside the world YZ tangent plane.

    Attributes:
        close_duration: Duration of the closing ramp (s).
        support_settle_duration: Settling time while the support is active (s).
        hold_duration: Unsupported hold time after the support is released (s).
        disturbance_duration: Duration of the tangential disturbance (s).
        recovery_duration: Observation time after the disturbance ends (s).
        force_n: Disturbance force amplitude (N).
        frequency_hz: Sinusoidal disturbance frequency (Hz).
        rotation_rate_rad_s: Angular velocity of the force direction in the YZ
            plane (rad/s); 0 keeps the force along world Y.
        slip_threshold_m: Tangential displacement threshold that defines slip (m).
    """

    close_duration: float = 1.0
    support_settle_duration: float = 0.5
    hold_duration: float = 0.5
    disturbance_duration: float = 1.0
    recovery_duration: float = 0.5
    force_n: float = 5.0
    frequency_hz: float = 2.0
    rotation_rate_rad_s: float = 0.0
    slip_threshold_m: float = 0.002

    def __post_init__(self) -> None:
        """Reject non-positive durations and negative amplitudes."""
        positive = {
            "close_duration": self.close_duration,
            "disturbance_duration": self.disturbance_duration,
            "frequency_hz": self.frequency_hz,
            "slip_threshold_m": self.slip_threshold_m,
        }
        nonnegative = {
            "support_settle_duration": self.support_settle_duration,
            "hold_duration": self.hold_duration,
            "recovery_duration": self.recovery_duration,
            "force_n": self.force_n,
            "rotation_rate_rad_s": self.rotation_rate_rad_s,
        }
        for name, value in positive.items():
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        for name, value in nonnegative.items():
            if value < 0:
                raise ValueError(f"{name} must be nonnegative")

    @property
    def release_time(self) -> float:
        """Time at which the temporary support is removed."""
        return self.close_duration + self.support_settle_duration

    @property
    def disturbance_start(self) -> float:
        """Time at which the tangential disturbance begins."""
        return self.release_time + self.hold_duration

    @property
    def disturbance_end(self) -> float:
        """Time at which the tangential disturbance ends."""
        return self.disturbance_start + self.disturbance_duration

    @property
    def total_duration(self) -> float:
        """Total simulated experiment duration (s)."""
        return self.disturbance_end + self.recovery_duration

    def phase_at(self, time_s: float) -> str:
        """Return the experiment phase containing ``time_s``."""
        if time_s < self.close_duration:
            return "close"
        if time_s < self.release_time:
            return "support_settle"
        if time_s < self.disturbance_start:
            return "unsupported_hold"
        if time_s < self.disturbance_end:
            return "disturbance"
        return "recovery"

    def force_vector_at(self, time_s: float) -> np.ndarray:
        """Return the world-frame 3D force applied at the object center of mass."""
        if not self.disturbance_start <= time_s < self.disturbance_end:
            return np.zeros(3, dtype=np.float64)
        elapsed = time_s - self.disturbance_start
        magnitude = self.force_n * math.sin(2.0 * math.pi * self.frequency_hz * elapsed)
        angle = self.rotation_rate_rad_s * elapsed
        return np.array(
            [0.0, magnitude * math.cos(angle), magnitude * math.sin(angle)],
            dtype=np.float64,
        )

    def force_y_at(self, time_s: float) -> float:
        """Return the world-Y component of the disturbance force."""
        return float(self.force_vector_at(time_s)[1])

    def step(
        self,
        model,
        data,
        *,
        actuator_id: int,
        close_control: float,
        support_geom_id: int,
        cube_body_id: int,
    ) -> Vector3:
        """Apply this step's control, support state, and object force.

        The drive target ramps linearly over ``close_duration``.  After
        ``release_time`` the support geom leaves the collision filter by
        zeroing its ``contype``/``conaffinity``, so support friction cannot
        resist the tangential disturbance.  The world-frame force is written
        into the first three components of ``xfrc_applied`` and returned.

        Returns:
            The applied world-frame force vector.
        """
        time_s = float(data.time)
        data.ctrl[actuator_id] = close_control * min(1.0, time_s / self.close_duration)
        if time_s >= self.release_time:
            model.geom_contype[support_geom_id] = 0
            model.geom_conaffinity[support_geom_id] = 0
        data.xfrc_applied[cube_body_id] = 0.0
        applied = self.force_vector_at(time_s)
        data.xfrc_applied[cube_body_id, :3] = applied
        return (float(applied[0]), float(applied[1]), float(applied[2]))
