"""
quadsim.types — Data classes for QuadSim responses.

These wrap the raw dictionary responses from the RPC layer into
typed, dot-accessible objects.
"""

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class SensorData:
    """IMU + GPS sensor readings from the drone."""

    imu_ang_vel: Tuple[float, float, float]
    imu_attitude: Tuple[float, float, float]
    imu_accel: Tuple[float, float, float]
    imu_vel: Tuple[float, float, float]
    imu_orientation: Tuple[float, float, float, float]
    imu_timestamp: float
    imu_valid: bool

    gps_position: Tuple[float, float, float]
    gps_timestamp: float
    gps_valid: bool

    @classmethod
    def from_dict(cls, d: dict) -> "SensorData":
        return cls(
            imu_ang_vel=tuple(d["imu_ang_vel"]),
            imu_attitude=tuple(d["imu_attitude"]),
            imu_accel=tuple(d["imu_accel"]),
            imu_vel=tuple(d["imu_vel"]),
            imu_orientation=tuple(d["imu_orientation"]),
            imu_timestamp=float(d["imu_timestamp"]),
            imu_valid=bool(d["imu_valid"]),
            gps_position=tuple(d["gps_position"]),
            gps_timestamp=float(d["gps_timestamp"]),
            gps_valid=bool(d["gps_valid"]),
        )


@dataclass(frozen=True)
class Telemetry:
    """Controller state and motor outputs from the drone."""

    drone_id: str
    mode: str
    controller: str
    motors: Tuple[float, float, float, float]
    desired_rates_deg: Tuple[float, float, float]
    desired_angles_deg: Tuple[float, float, float]
    desired_vel: Tuple[float, float, float]
    external_cmd: Tuple[float, float, float, float]

    @classmethod
    def from_dict(cls, d: dict) -> "Telemetry":
        return cls(
            drone_id=str(d.get("drone_id", "")),
            mode=str(d.get("mode", "unknown")),
            controller=str(d.get("controller", "unknown")),
            motors=tuple(d["motors"]),
            desired_rates_deg=tuple(d["desired_rates_deg"]),
            desired_angles_deg=tuple(d["desired_angles_deg"]),
            desired_vel=tuple(d["desired_vel"]),
            external_cmd=tuple(d["external_cmd"]),
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
    def from_dict(cls, d: dict) -> "SimStatus":
        return cls(
            is_paused=bool(d["is_paused"]),
            time_scale=float(d["time_scale"]),
            sim_time=float(d["sim_time"]),
            fixed_dt=float(d["fixed_dt"]),
            authority=str(d["authority"]),
            ui_status=str(d["ui_status"]),
            internal_status=str(d["internal_status"]),
            external_status=str(d["external_status"]),
            client_connected=bool(d["client_connected"]),
            client_name=str(d.get("client_name", "")),
        )