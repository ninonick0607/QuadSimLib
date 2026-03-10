# quadsim/_control_loops.py
# Internal Control Loop Implementations
#
# PURPOSE:
#   Pure control logic for high-level flight commands. Each factory
#   function returns a tick callable that reads sensors, computes the
#   next command, sends it, and returns True when done.
#
#   These take a Drone instance — all RPC calls go directly through it.
#
# COORDINATE FRAME:
#   Everything is FLU (X forward, Y left, Z up).
#   gps_position = (x, y, z) where Z is altitude.
#   Velocity: (vx, vy, vz) where vz is vertical.
#   Position: (x, y, z, yaw) where z is altitude.

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Callable, List, Tuple

if TYPE_CHECKING:
    from .drone import Drone


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _distance_3d(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _distance_horizontal(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    """Horizontal (XY plane) distance. Z is altitude in FLU."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


# ============================================================================
# Takeoff
# ============================================================================

def make_takeoff_loop(drone: Drone, target_altitude: float, speed: float) -> Callable[[], bool]:
    """
    Climb to target_altitude using velocity mode, then switch to position hold.
    """
    mode_set = False
    hold_started = False

    def tick() -> bool:
        nonlocal mode_set, hold_started

        sensors = drone.get_sensors()
        pos = sensors.gps_position
        current_alt = pos[2]  # Z is altitude in FLU

        if not mode_set:
            drone.set_mode("velocity")
            mode_set = True

        if not hold_started:
            error = target_altitude - current_alt

            if abs(error) < drone.altitude_tolerance:
                drone.set_mode("position")
                drone.send_command(x=pos[0], y=pos[1], z=target_altitude, w=0.0)
                hold_started = True
                return False

            climb_rate = _clamp(error * 1.0, speed)
            drone.send_command(x=0, y=0, z=climb_rate, w=0)
            return False
        else:
            drone.send_command(x=pos[0], y=pos[1], z=target_altitude, w=0.0)
            return abs(target_altitude - current_alt) < drone.altitude_tolerance

    return tick


# ============================================================================
# Land
# ============================================================================

def make_land_loop(drone: Drone, speed: float) -> Callable[[], bool]:
    """
    Descend using velocity mode. Cut throttle when near ground.
    """
    mode_set = False

    def tick() -> bool:
        nonlocal mode_set

        if not mode_set:
            drone.set_mode("velocity")
            mode_set = True

        sensors = drone.get_sensors()
        current_alt = sensors.gps_position[2]

        if current_alt < drone.landing_altitude:
            drone.set_mode("rate")
            drone.send_command(x=0, y=0, z=0, w=0)
            return True

        descend_rate = -min(speed, max(0.5, current_alt * 0.5))
        drone.send_command(x=0, y=0, z=descend_rate, w=0)
        return False

    return tick


# ============================================================================
# Hover  (Step 5: now captures and preserves current yaw)
# ============================================================================

def make_hover_loop(drone: Drone, duration: float) -> Callable[[], bool]:
    """
    Capture current position AND heading, hold both for duration seconds.
    """
    import time as _time

    mode_set = False
    start_time = None
    hold_pos = None
    hold_yaw = None

    def tick() -> bool:
        nonlocal mode_set, start_time, hold_pos, hold_yaw

        if not mode_set:
            sensors = drone.get_sensors()
            hold_pos = sensors.gps_position
            hold_yaw = sensors.imu_attitude[2]  # Current yaw in degrees
            drone.set_mode("position")
            mode_set = True
            start_time = _time.time()

        drone.send_command(x=hold_pos[0], y=hold_pos[1], z=hold_pos[2], w=hold_yaw)
        return _time.time() - start_time >= duration

    return tick


# ============================================================================
# Fly To (position mode)
# ============================================================================

def make_fly_to_loop(
    drone: Drone,
    target_x: float, target_y: float, target_z: float,
    speed: float, yaw: float,
) -> Callable[[], bool]:
    """
    Send position setpoint until within tolerance.
    """
    mode_set = False
    target = (target_x, target_y, target_z)

    def tick() -> bool:
        nonlocal mode_set

        if not mode_set:
            drone.set_mode("position")
            mode_set = True

        drone.send_command(x=target_x, y=target_y, z=target_z, w=yaw)

        pos = drone.get_sensors().gps_position
        return _distance_3d(pos, target) < drone.position_tolerance

    return tick


# ============================================================================
# Fly To (velocity mode fallback)
# ============================================================================

def make_fly_to_velocity_loop(
    drone: Drone,
    target_x: float, target_y: float, target_z: float,
    speed: float, yaw_rate: float,
) -> Callable[[], bool]:
    """
    Compute velocity vector toward target, send as velocity command.
    """
    mode_set = False
    target = (target_x, target_y, target_z)

    def tick() -> bool:
        nonlocal mode_set

        if not mode_set:
            drone.set_mode("velocity")
            mode_set = True

        pos = drone.get_sensors().gps_position
        dx = target_x - pos[0]
        dy = target_y - pos[1]
        dz = target_z - pos[2]

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < drone.position_tolerance:
            drone.send_command(x=0, y=0, z=0, w=0)
            return True

        scale = min(speed, dist) / max(dist, 1e-6)
        drone.send_command(x=dx * scale, y=dy * scale, z=dz * scale, w=yaw_rate)
        return False

    return tick


# ============================================================================
# Yaw To
# ============================================================================

def make_yaw_to_loop(drone: Drone, target_heading_deg: float, rate: float) -> Callable[[], bool]:
    """
    Hold position, set yaw to target heading. Done when within 2 degrees.
    """
    hold_pos = None
    yaw_tolerance = 2.0

    def tick() -> bool:
        nonlocal hold_pos

        sensors = drone.get_sensors()

        if hold_pos is None:
            hold_pos = sensors.gps_position
            drone.set_mode("position")

        drone.send_command(x=hold_pos[0], y=hold_pos[1], z=hold_pos[2], w=target_heading_deg)

        current_yaw = sensors.imu_attitude[2]
        error = (target_heading_deg - current_yaw + 180.0) % 360.0 - 180.0
        return abs(error) < yaw_tolerance

    return tick


# ============================================================================
# Fly Path
# ============================================================================

def make_fly_path_loop(
    drone: Drone,
    waypoints: List[Tuple[float, float, float]],
    speed: float,
    per_leg_timeout: float,
) -> Callable[[], bool]:
    """
    Fly through waypoints in order. Per-leg timeout catches stalls.
    """
    import time as _time
    from .exceptions import TimeoutError

    current_index = 0
    current_leg_fn = None
    leg_start_time = None

    def tick() -> bool:
        nonlocal current_index, current_leg_fn, leg_start_time

        if current_index >= len(waypoints):
            return True

        if current_leg_fn is None:
            wp = waypoints[current_index]
            current_leg_fn = make_fly_to_loop(drone, wp[0], wp[1], wp[2], speed, yaw=0.0)
            leg_start_time = _time.monotonic()

        if _time.monotonic() - leg_start_time > per_leg_timeout:
            raise TimeoutError(f"Waypoint {current_index} not reached within {per_leg_timeout}s")

        if current_leg_fn():
            current_index += 1
            current_leg_fn = None
            leg_start_time = None
            return current_index >= len(waypoints)

        return False

    return tick