# quadsim/types.py
# Typed Data Structures
#
# COORDINATE FRAME:
#   All sensor data is FLU (X forward, Y left, Z up).
#   GPS position: (x, y, z) where Z is altitude.
#   IMU attitude: (roll, pitch, yaw) degrees.
#   IMU angular velocity: rad/s.
#   IMU acceleration: m/s².
#   IMU velocity: m/s.
#   IMU orientation: quaternion (x, y, z, w).

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


def _vec3(raw) -> Tuple[float, float, float]:
    if raw is None:
        return (0.0, 0.0, 0.0)
    return (float(raw[0]), float(raw[1]), float(raw[2]))


def _vec4(raw) -> Tuple[float, float, float, float]:
    if raw is None:
        return (0.0, 0.0, 0.0, 0.0)
    return (float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3]))


def _motor4(raw) -> Tuple[float, float, float, float]:
    if raw is None:
        return (0.0, 0.0, 0.0, 0.0)
    return (float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3]))


@dataclass(frozen=True)
class SensorData:
    """Immutable snapshot of drone sensor readings."""
    imu_ang_vel: Tuple[float, float, float]
    imu_attitude: Tuple[float, float, float]
    imu_accel: Tuple[float, float, float]
    imu_vel: Tuple[float, float, float]
    imu_orientation: Tuple[float, float, float, float]
    imu_timestamp: float
    imu_valid: bool
    gps_position: Tuple[float, float, float]    # FLU world, Z is altitude
    gps_timestamp: float
    gps_valid: bool

    @classmethod
    def from_dict(cls, d: dict) -> SensorData:
        return cls(
            imu_ang_vel=_vec3(d.get("imu_ang_vel")),
            imu_attitude=_vec3(d.get("imu_attitude")),
            imu_accel=_vec3(d.get("imu_accel")),
            imu_vel=_vec3(d.get("imu_vel")),
            imu_orientation=_vec4(d.get("imu_orientation")),
            imu_timestamp=float(d.get("imu_timestamp", 0)),
            imu_valid=bool(d.get("imu_valid", False)),
            gps_position=_vec3(d.get("gps_position")),
            gps_timestamp=float(d.get("gps_timestamp", 0)),
            gps_valid=bool(d.get("gps_valid", False)),
        )


@dataclass(frozen=True)
class Telemetry:
    """Operational telemetry snapshot."""
    drone_id: str
    mode: str
    controller: str
    motors: Tuple[float, float, float, float]
    desired_rates_deg: Tuple[float, float, float]
    desired_angles_deg: Tuple[float, float, float]
    desired_vel: Tuple[float, float, float]
    external_cmd: Tuple[float, float, float, float]

    @classmethod
    def from_dict(cls, d: dict) -> Telemetry:
        return cls(
            drone_id=str(d.get("drone_id", "")),
            mode=str(d.get("mode", "unknown")),
            controller=str(d.get("controller", "unknown")),
            motors=_motor4(d.get("motors")),
            desired_rates_deg=_vec3(d.get("desired_rates_deg")),
            desired_angles_deg=_vec3(d.get("desired_angles_deg")),
            desired_vel=_vec3(d.get("desired_vel")),
            external_cmd=_vec4(d.get("external_cmd")),
        )


@dataclass(frozen=True)
class SimStatus:
    """Simulation-wide status snapshot."""
    is_paused: bool
    time_scale: float
    sim_time: float
    fixed_dt: float
    authority: str
    ui_status: str
    internal_status: str
    external_status: str
    client_connected: bool
    client_name: str

    @classmethod
    def from_dict(cls, d: dict) -> SimStatus:
        return cls(
            is_paused=bool(d.get("is_paused", False)),
            time_scale=float(d.get("time_scale", 1.0)),
            sim_time=float(d.get("sim_time", 0.0)),
            fixed_dt=float(d.get("fixed_dt", 0.02)),
            authority=str(d.get("authority", "unknown")),
            ui_status=str(d.get("ui_status", "unknown")),
            internal_status=str(d.get("internal_status", "unknown")),
            external_status=str(d.get("external_status", "unknown")),
            client_connected=bool(d.get("client_connected", False)),
            client_name=str(d.get("client_name", "")),
        )