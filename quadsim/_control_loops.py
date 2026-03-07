# quadsim/_control_loops.py
# Phase 10: Internal Control Loop Implementations
#
# PURPOSE:
#   Pure control logic for high-level flight commands. Each factory function
#   returns a tick callable that:
#     - Reads sensors from the client
#     - Computes the next command
#     - Sends the command
#     - Returns True when the maneuver is complete
#
# DESIGN:
#   Separated from api.py so the QuadSimApi class stays focused on
#   public interface, threading, and state management. These functions
#   are independently testable — pass a mock client and verify the
#   command sequence.
#
# COORDINATE FRAME:
#   All positions are Unity world coordinates (Y-up, left-handed).
#   gps_position = (x, y, z) where Y is altitude.
#   Velocity commands: (vx, vy, vz) where vy is vertical.
#   Position commands: (x, y, z, yaw) matching Axis4 semantics.
#
# AXIS4 SEMANTICS REMINDER (from handoff):
#   Rate:     (roll_rate°/s, pitch_rate°/s, yaw_rate°/s, throttle[0,1])
#   Angle:    (roll°, pitch°, yaw_rate°/s, throttle[0,1])
#   Velocity: (vx m/s, vy m/s, vz m/s, yaw_rate°/s)
#   Position: (x m, y m, z m, yaw°)

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Callable, List, Tuple

if TYPE_CHECKING:
    from .api import QuadSimApi


def _clamp(value: float, limit: float) -> float:
    """Clamp value to [-limit, +limit]."""
    return max(-limit, min(limit, value))


def _distance_3d(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    """Euclidean distance between two 3D points."""
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def _distance_horizontal(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    """Horizontal (XZ plane) distance between two 3D points. Y is altitude."""
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[2] - b[2]) ** 2
    )


# ============================================================================
# Takeoff
# ============================================================================

def make_takeoff_loop(
    api: QuadSimApi,
    target_altitude: float,
    speed: float,
) -> Callable[[], bool]:
    """
    Create a tick function for takeoff.

    Strategy:
    1. Use velocity mode to climb at `speed` m/s.
    2. Proportional ramp-down as we approach the target.
    3. Once within altitude_tolerance, switch to position hold.

    Args:
        api: The QuadSimApi instance (provides client + config).
        target_altitude: Desired altitude in meters (Y-up).
        speed: Maximum climb rate in m/s.

    Returns:
        Callable that returns True when takeoff is complete.
    """
    # Track whether we've done the initial mode switch
    mode_set = False
    # Capture starting XZ position so we don't drift during climb
    start_pos = None

    def tick() -> bool:
        nonlocal mode_set, start_pos

        sensors = api.client.get_sensors()
        pos = sensors.gps_position  # (x, y, z)
        current_alt = pos[1]

        if start_pos is None:
            start_pos = pos

        if not mode_set:
            api.client.set_mode("velocity")
            mode_set = True

        error = target_altitude - current_alt

        if abs(error) < api.altitude_tolerance:
            # Arrived — switch to position hold at target altitude
            api.client.send_command(
                x=start_pos[0], y=target_altitude, z=start_pos[2], w=0.0,
                mode="position",
            )
            return True

        # Proportional climb rate, clamped to max speed
        # Ramp down when within 2x tolerance to avoid overshoot
        climb_rate = _clamp(error, speed)

        api.client.send_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=climb_rate)
        return False

    return tick


# ============================================================================
# Land
# ============================================================================

def make_land_loop(
    api: QuadSimApi,
    speed: float,
) -> Callable[[], bool]:
    """
    Create a tick function for landing.

    Strategy:
    1. Use velocity mode with negative vertical velocity.
    2. Reduce speed as altitude decreases for soft touchdown.
    3. Detect ground when altitude < landing_altitude.

    Args:
        api: The QuadSimApi instance.
        speed: Maximum descent rate in m/s (positive value; will be negated).

    Returns:
        Callable that returns True when landing is complete.
    """
    mode_set = False

    def tick() -> bool:
        nonlocal mode_set

        sensors = api.client.get_sensors()
        pos = sensors.gps_position
        current_alt = pos[1]

        if not mode_set:
            api.client.set_mode("velocity")
            mode_set = True

        # Ground detection
        if current_alt <= api.landing_altitude:
            # On the ground — zero velocity and hold
            api.client.send_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
            return True

        # Proportional descent: slow down as we approach the ground
        # Full speed above 2m, proportional below that
        descent_rate = speed
        if current_alt < 2.0:
            # Scale: at 2m use full speed, at 0m use 20% speed
            fraction = max(0.2, current_alt / 2.0)
            descent_rate = speed * fraction

        # Negative vy = descend (Y-up frame)
        api.client.send_command(vx=0.0, vy=-descent_rate, vz=0.0, yaw_rate=0.0)
        return False

    return tick


# ============================================================================
# Hover (hold position for duration)
# ============================================================================

def make_hover_loop(
    api: QuadSimApi,
    duration: float,
) -> Callable[[], bool]:
    """
    Create a tick function for position hold.

    Strategy:
    1. Capture current position on first tick.
    2. Switch to position mode.
    3. Send position setpoint each tick.
    4. Return True after `duration` seconds.

    Uses time.monotonic internally for wall-clock timing.
    If the sim is paused, the timer still counts — this is intentional,
    as hover is typically a wall-clock operation in user scripts.

    Args:
        api: The QuadSimApi instance.
        duration: How long to hold in seconds (wall-clock).

    Returns:
        Callable that returns True when duration has elapsed.
    """
    import time

    hold_pos = None
    start_time = None

    def tick() -> bool:
        nonlocal hold_pos, start_time

        if hold_pos is None:
            sensors = api.client.get_sensors()
            hold_pos = sensors.gps_position
            start_time = time.monotonic()
            api.client.set_mode("position")

        # Keep sending the hold setpoint
        api.client.send_command(
            x=hold_pos[0], y=hold_pos[1], z=hold_pos[2], w=0.0,
        )

        elapsed = time.monotonic() - start_time
        return elapsed >= duration

    return tick


# ============================================================================
# Fly To (navigate to target position)
# ============================================================================

def make_fly_to_loop(
    api: QuadSimApi,
    target_x: float,
    target_y: float,
    target_z: float,
    speed: float,
    yaw: float,
) -> Callable[[], bool]:
    """
    Create a tick function for flying to a world position.

    Strategy:
    There are two approaches depending on whether the position controller
    is working:

    APPROACH A (position mode — preferred):
      Set mode to position, send the target as a setpoint, let the
      cascaded controller handle it. Monitor GPS until within tolerance.

    APPROACH B (velocity mode — fallback):
      Compute velocity vector toward target, clamp to speed limit,
      send as velocity command. More work on our side but doesn't
      depend on a tuned position controller.

    This implementation uses APPROACH A because the Phase 9 handoff
    confirms the cascaded controller supports GoalMode.Position.
    If position mode doesn't work well in practice, switch to
    APPROACH B by replacing the body of tick().

    Args:
        api: The QuadSimApi instance.
        target_x, target_y, target_z: Target position in Unity world coords.
        speed: Desired approach speed in m/s. Note: in position mode the
               controller handles speed internally. This param is stored
               for a future velocity-mode fallback.
        yaw: Target yaw in degrees.

    Returns:
        Callable that returns True when within position_tolerance.
    """
    mode_set = False
    target = (target_x, target_y, target_z)

    def tick() -> bool:
        nonlocal mode_set

        if not mode_set:
            api.client.set_mode("position")
            mode_set = True

        # Send position setpoint
        api.client.send_command(
            x=target_x, y=target_y, z=target_z, w=yaw,
        )

        # Check arrival
        sensors = api.client.get_sensors()
        pos = sensors.gps_position
        dist = _distance_3d(pos, target)

        return dist < api.position_tolerance

    return tick


def make_fly_to_velocity_loop(
    api: QuadSimApi,
    target_x: float,
    target_y: float,
    target_z: float,
    speed: float,
    yaw_rate: float,
) -> Callable[[], bool]:
    """
    Velocity-mode fallback for fly_to.

    Use this if the position controller isn't tuned or functional.
    Computes a velocity vector toward the target, clamped to `speed`,
    and sends it as a velocity command.

    Args:
        api: The QuadSimApi instance.
        target_x, target_y, target_z: Target in Unity world coords.
        speed: Max speed in m/s.
        yaw_rate: Yaw rate in °/s during transit.

    Returns:
        Callable that returns True when within position_tolerance.
    """
    mode_set = False
    target = (target_x, target_y, target_z)

    def tick() -> bool:
        nonlocal mode_set

        if not mode_set:
            api.client.set_mode("velocity")
            mode_set = True

        sensors = api.client.get_sensors()
        pos = sensors.gps_position
        dist = _distance_3d(pos, target)

        if dist < api.position_tolerance:
            # Arrived — zero velocity
            api.client.send_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
            return True

        # Unit vector toward target, scaled by speed
        # Ramp down within 2x tolerance
        effective_speed = speed
        if dist < api.position_tolerance * 3.0:
            effective_speed = speed * (dist / (api.position_tolerance * 3.0))
            effective_speed = max(effective_speed, 0.1)  # Don't stall

        dx = target_x - pos[0]
        dy = target_y - pos[1]
        dz = target_z - pos[2]

        # Normalize
        vx = (dx / dist) * effective_speed
        vy = (dy / dist) * effective_speed
        vz = (dz / dist) * effective_speed

        api.client.send_command(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)
        return False

    return tick


# ============================================================================
# Yaw To (rotate to heading)
# ============================================================================

def make_yaw_to_loop(
    api: QuadSimApi,
    target_heading_deg: float,
    rate: float,
) -> Callable[[], bool]:
    """
    Create a tick function for rotating to an absolute heading.

    Strategy:
    1. Hold current position in position mode.
    2. Set the W component (yaw) to the target heading.
    3. Monitor IMU attitude yaw until within tolerance.

    Args:
        api: The QuadSimApi instance.
        target_heading_deg: Target yaw in degrees.
        rate: Not directly usable in position mode (controller handles it).
              Stored for a potential rate-mode fallback.

    Returns:
        Callable that returns True when yaw is within 2 degrees.
    """
    hold_pos = None
    yaw_tolerance = 2.0  # degrees

    def tick() -> bool:
        nonlocal hold_pos

        sensors = api.client.get_sensors()

        if hold_pos is None:
            hold_pos = sensors.gps_position
            api.client.set_mode("position")

        # Send position hold with target yaw
        api.client.send_command(
            x=hold_pos[0], y=hold_pos[1], z=hold_pos[2],
            w=target_heading_deg,
        )

        # Check yaw convergence
        current_yaw = sensors.imu_attitude[2]  # (roll, pitch, yaw)

        # Handle wraparound: compute shortest angular distance
        error = target_heading_deg - current_yaw
        # Normalize to [-180, 180]
        error = (error + 180.0) % 360.0 - 180.0

        return abs(error) < yaw_tolerance

    return tick


# ============================================================================
# Fly Path (sequence of waypoints)
# ============================================================================

def make_fly_path_loop(
    api: QuadSimApi,
    waypoints: List[Tuple[float, float, float]],
    speed: float,
    per_leg_timeout: float,
) -> Callable[[], bool]:
    """
    Create a tick function for following a waypoint path.

    Strategy:
    1. For each waypoint, create a fly_to inner loop.
    2. Run the inner loop until the waypoint is reached or per_leg_timeout.
    3. Advance to the next waypoint.
    4. Return True when all waypoints are visited.

    Args:
        api: The QuadSimApi instance.
        waypoints: List of (x, y, z) positions in Unity world coords.
        speed: Max speed in m/s.
        per_leg_timeout: Max seconds for each leg. Raises TimeoutError if exceeded.

    Returns:
        Callable that returns True when all waypoints are reached.
    """
    import time
    from .exceptions import TimeoutError

    current_index = 0
    current_leg_fn = None
    leg_start_time = None

    def tick() -> bool:
        nonlocal current_index, current_leg_fn, leg_start_time

        if current_index >= len(waypoints):
            return True

        # Initialize leg if needed
        if current_leg_fn is None:
            wp = waypoints[current_index]
            current_leg_fn = make_fly_to_loop(
                api, wp[0], wp[1], wp[2], speed, yaw=0.0,
            )
            leg_start_time = time.monotonic()

        # Check per-leg timeout
        if time.monotonic() - leg_start_time > per_leg_timeout:
            raise TimeoutError(
                f"Waypoint {current_index} not reached within {per_leg_timeout}s"
            )

        # Run the inner loop
        leg_done = current_leg_fn()

        if leg_done:
            current_index += 1
            current_leg_fn = None
            leg_start_time = None

            if current_index >= len(waypoints):
                return True

        return False

    return tick