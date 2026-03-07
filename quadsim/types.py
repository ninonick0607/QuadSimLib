# quadsim/types.py
# Phase 9d: Typed Data Structures
#
# PURPOSE:
#   Frozen dataclasses representing simulation data snapshots.
#   Each has a from_dict() classmethod that parses the raw MessagePack
#   dictionary returned by the server.
#
# COORDINATE FRAME:
#   GPS position is in Unity world coordinates (Y-up, left-handed).
#   IMU attitude is (roll, pitch, yaw) in degrees, FLU convention.
#   IMU angular velocity is in rad/s, FLU convention.
#   IMU acceleration is in m/s², FLU convention.
#   IMU velocity is in m/s, FLU convention.
#   IMU orientation is a quaternion (x, y, z, w).
#
# TUPLE COERCION:
#   Vector3 arrays from the wire are coerced to tuple[float, float, float].
#   Quaternion arrays are coerced to tuple[float, float, float, float].
#   This makes them hashable and prevents accidental mutation.

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


def _vec3(raw) -> Tuple[float, float, float]:
    """Coerce a list/array from the wire into a 3-tuple of floats."""
    if raw is None:
        return (0.0, 0.0, 0.0)
    return (float(raw[0]), float(raw[1]), float(raw[2]))


def _vec4(raw) -> Tuple[float, float, float, float]:
    """Coerce a list/array from the wire into a 4-tuple of floats."""
    if raw is None:
        return (0.0, 0.0, 0.0, 0.0)
    return (float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3]))


def _motor4(raw) -> Tuple[float, float, float, float]:
    """Coerce motor outputs (FL, FR, BL, BR) into a 4-tuple."""
    if raw is None:
        return (0.0, 0.0, 0.0, 0.0)
    return (float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3]))


@dataclass(frozen=True)
class SensorData:
    """
    Immutable snapshot of drone sensor readings.

    Matches the wire format from ExternalRpcAdapter.HandleGetSensorData().
    All vectors are tuples to ensure immutability and hashability.
    """
    imu_ang_vel: Tuple[float, float, float]         # rad/s, FLU
    imu_attitude: Tuple[float, float, float]         # degrees (roll, pitch, yaw), FLU
    imu_accel: Tuple[float, float, float]            # m/s², FLU
    imu_vel: Tuple[float, float, float]              # m/s, FLU
    imu_orientation: Tuple[float, float, float, float]  # quaternion (x, y, z, w)
    imu_timestamp: float                              # seconds
    imu_valid: bool
    gps_position: Tuple[float, float, float]         # meters, Unity world (Y-up)
    gps_timestamp: float                              # seconds
    gps_valid: bool

    @classmethod
    def from_dict(cls, d: dict) -> SensorData:
        """Parse from the raw MessagePack dictionary returned by the server."""
        return cls(
            imu_ang_vel=_vec3(d.get("imu_ang_vel")),
            imu_attitude=_vec3(d.get("imu_attitude")),
            imu_accel=_vec3(d.get("imu_accel")),
            imu_vel=_vec3(d.get("imu_vel")),
            imu_orientation=_vec4(d.get("imu_orientation")),
            imu_timestamp=float(d.get("imu_timestamp", 0.0)),
            imu_valid=bool(d.get("imu_valid", False)),
            gps_position=_vec3(d.get("gps_position")),
            gps_timestamp=float(d.get("gps_timestamp", 0.0)),
            gps_valid=bool(d.get("gps_valid", False)),
        )


@dataclass(frozen=True)
class Telemetry:
    """
    Immutable snapshot of drone operational state and controller outputs.

    Matches the wire format from ExternalRpcAdapter.HandleGetTelemetry().
    """
    drone_id: str
    mode: str                                          # GoalMode string
    controller: str                                    # ControllerKind string
    motors: Tuple[float, float, float, float]          # FL, FR, BL, BR [0..1]
    desired_rates_deg: Tuple[float, float, float]      # degrees/s
    desired_angles_deg: Tuple[float, float, float]     # degrees
    desired_vel: Tuple[float, float, float]            # m/s
    external_cmd: Tuple[float, float, float, float]    # Axis4 (x, y, z, w)

    @classmethod
    def from_dict(cls, d: dict) -> Telemetry:
        """Parse from the raw MessagePack dictionary returned by the server."""
        return cls(
            drone_id=str(d.get("drone_id", "")),
            mode=str(d.get("mode", "none")),
            controller=str(d.get("controller", "cascade")),
            motors=_motor4(d.get("motors")),
            desired_rates_deg=_vec3(d.get("desired_rates_deg")),
            desired_angles_deg=_vec3(d.get("desired_angles_deg")),
            desired_vel=_vec3(d.get("desired_vel")),
            external_cmd=_vec4(d.get("external_cmd")),
        )


@dataclass(frozen=True)
class SimStatus:
    """
    Immutable snapshot of simulation-wide state.

    Matches the wire format from ExternalRpcAdapter.HandleGetStatus().
    """
    is_paused: bool
    time_scale: float
    sim_time: float               # seconds
    fixed_dt: float               # seconds per physics step
    authority: str                 # "ui", "internal", "external"
    ui_status: str                 # SourceStatus string
    internal_status: str           # SourceStatus string
    external_status: str           # SourceStatus string
    client_connected: bool
    client_name: str

    @classmethod
    def from_dict(cls, d: dict) -> SimStatus:
        """Parse from the raw MessagePack dictionary returned by the server."""
        return cls(
            is_paused=bool(d.get("is_paused", False)),
            time_scale=float(d.get("time_scale", 1.0)),
            sim_time=float(d.get("sim_time", 0.0)),
            fixed_dt=float(d.get("fixed_dt", 0.02)),
            authority=str(d.get("authority", "ui")),
            ui_status=str(d.get("ui_status", "")),
            internal_status=str(d.get("internal_status", "")),
            external_status=str(d.get("external_status", "")),
            client_connected=bool(d.get("client_connected", False)),
            client_name=str(d.get("client_name", "")),
        )