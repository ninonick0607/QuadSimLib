# quadsim/drone.py
# Drone — Per-Drone Control Handle
#
# PURPOSE:
#   Single object for everything drone-related, both low-level
#   (set_mode, send_command, get_sensors) and high-level
#   (takeoff, fly_to, hover, land).
#
#   Does NOT own a connection — routes all RPC through a shared
#   Transport instance owned by QuadSim.
#
# COORDINATE FRAME:
#   Everything is FLU (X forward, Y left, Z up).
#   Z is altitude in all position/velocity contexts.

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from ._transport import Transport
from .exceptions import QuadSimError, TimeoutError
from .future import Future
from .types import SensorData, Telemetry
from . import _control_loops as loops


class Drone:
    """
    Per-drone control handle. Low-level and high-level on one object.

    Get this from ``QuadSim.drone()``, don't construct directly.

    Usage::

        sim = QuadSim()
        sim.connect()
        drone = sim.drone()

        # High-level
        drone.takeoff(3.0)
        drone.fly_to(5, 0, 3)

        # Low-level
        drone.set_mode("rate")
        drone.send_command(roll=0, pitch=0, yaw=30, throttle=0.44)
        sensors = drone.get_sensors()

        drone.land()
    """

    def __init__(self, transport: Transport):
        self._transport = transport

        # ── Tuning parameters (public) ──

        #: 3D distance threshold for "arrived" (meters).
        self.position_tolerance: float = 0.5

        #: Vertical distance threshold for altitude checks (meters).
        self.altitude_tolerance: float = 0.3

        #: Below this altitude = "on the ground" (meters).
        self.landing_altitude: float = 0.15

        #: Default speed for fly_to/fly_path if not specified (m/s).
        self.default_speed: float = 2.0

        #: Internal control loop tick rate (Hz).
        self.control_loop_hz: float = 50.0

        #: Default per-leg timeout for fly_path (seconds).
        self.default_leg_timeout: float = 30.0

        #: If True, fly_to uses velocity-mode guidance instead of position mode.
        self.use_velocity_mode_navigation: bool = False

        # ── Internal state ──
        self._active_future: Optional[Future] = None
        self._command_lock = threading.Lock()

    # ====================================================================
    # Low-Level: Mode & Controller
    # ====================================================================

    def set_mode(self, mode: str) -> str:
        """
        Set the drone's goal mode.

        Args:
            mode: "none", "rate", "angle", "velocity", "position",
                  "passthrough", "wrench"

        Returns:
            Confirmed mode string.
        """
        resp = self._transport.request("set_mode", {"mode": mode})
        return resp.get("mode", mode)

    def set_controller(self, controller: str) -> str:
        """
        Set the active controller type.

        Args:
            controller: "cascade" or "geometric"

        Returns:
            Confirmed controller string.
        """
        resp = self._transport.request("set_controller", {"controller": controller})
        return resp.get("controller", controller)

    # ====================================================================
    # Low-Level: Commands
    # ====================================================================

    def send_command(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        w: float = 0.0,
        *,
        roll: Optional[float] = None,
        pitch: Optional[float] = None,
        yaw: Optional[float] = None,
        yaw_rate: Optional[float] = None,
        throttle: Optional[float] = None,
        vx: Optional[float] = None,
        vy: Optional[float] = None,
        vz: Optional[float] = None,
        mode: Optional[str] = None,
    ) -> None:
        """
        Send a raw Axis4 command. FLU convention.

        Rate/Angle::
            send_command(roll=0, pitch=0, yaw=30, throttle=0.44)

        Velocity::
            send_command(vx=1.0, vy=0, vz=0.5, yaw_rate=0)

        Position::
            send_command(x=5, y=0, z=3, w=0)  # z is altitude

        With mode override::
            send_command(vx=1.0, vy=0, vz=0, yaw_rate=0, mode="velocity")
        """
        if roll is not None: x = roll
        if pitch is not None: y = pitch
        if yaw is not None: z = yaw
        if throttle is not None: w = throttle
        if vx is not None: x = vx
        if vy is not None: y = vy
        if vz is not None: z = vz
        if yaw_rate is not None: w = yaw_rate

        params: Dict[str, Any] = {"x": x, "y": y, "z": z, "w": w}
        if mode is not None:
            params["mode"] = mode

        self._transport.request("send_command", params)

    # ====================================================================
    # Low-Level: Sensors & Telemetry
    # ====================================================================

    def get_sensors(self) -> SensorData:
        """Full sensor snapshot (IMU + GPS)."""
        return SensorData.from_dict(self._transport.request("get_sensor_data"))

    def get_telemetry(self) -> Telemetry:
        """Controller state, motor outputs, desired setpoints."""
        return Telemetry.from_dict(self._transport.request("get_telemetry"))

    def get_position(self) -> Tuple[float, float, float]:
        """Current GPS position (x, y, z) in FLU. Z is altitude."""
        return self.get_sensors().gps_position

    def get_attitude(self) -> Tuple[float, float, float]:
        """Current attitude (roll, pitch, yaw) in degrees, FLU."""
        return self.get_sensors().imu_attitude

    def get_velocity(self) -> Tuple[float, float, float]:
        """Current velocity (vx, vy, vz) in m/s, FLU."""
        return self.get_sensors().imu_vel

    def is_airborne(self) -> bool:
        """True if altitude > landing_altitude."""
        pos = self.get_position()
        return pos[2] > self.landing_altitude

    # ====================================================================
    # Low-Level: Resets
    # ====================================================================

    def reset(self) -> None:
        """Full drone reset (pose + physics + controller)."""
        self._cancel_active()
        self._transport.request("reset_all")

    def reset_pose(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0,
    ) -> None:
        """Teleport the drone to a specific pose."""
        self._transport.request("reset_pose", {
            "x": x, "y": y, "z": z,
            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
        })

    def reset_rotation(self) -> None:
        """Level the drone (zero roll/pitch), zero velocities."""
        self._transport.request("reset_rotation")

    def reset_physics(self) -> None:
        """Zero all velocities without moving."""
        self._transport.request("reset_physics")

    def reset_controller(self) -> None:
        """Clear PID integrators and controller internal state."""
        self._transport.request("reset_controller")

    # ====================================================================
    # Low-Level: Streaming
    # ====================================================================

    def subscribe_sensors(
        self,
        callback: Callable[[SensorData], None],
        hz: float = 50.0,
    ) -> None:
        """Push-based sensor data at configurable rate."""
        def _typed(topic: str, data: dict) -> None:
            if topic == "sensors":
                callback(SensorData.from_dict(data))

        self._transport.subscribe(_typed, topics=["sensors"], hz=hz)

    def subscribe_telemetry(
        self,
        callback: Callable[[Telemetry], None],
        hz: float = 50.0,
    ) -> None:
        """Push-based telemetry data at configurable rate."""
        def _typed(topic: str, data: dict) -> None:
            if topic == "telemetry":
                callback(Telemetry.from_dict(data))

        self._transport.subscribe(_typed, topics=["telemetry"], hz=hz)

    def subscribe(
        self,
        callback: Callable,
        topics: Optional[List[str]] = None,
        hz: float = 50.0,
    ) -> None:
        """Raw streaming subscription. Callback gets (topic, data_dict)."""
        self._transport.subscribe(callback, topics=topics, hz=hz)

    def unsubscribe(self) -> None:
        """Stop streaming. Idempotent."""
        self._transport.unsubscribe()

    # ====================================================================
    # High-Level: Blocking Flight Commands
    # ====================================================================

    def takeoff(
        self,
        altitude: float = 3.0,
        speed: float = 1.0,
        timeout: float = 30.0,
    ) -> None:
        """
        Take off to altitude and stabilize. Blocks until done.

        Args:
            altitude: Target altitude in meters (Z-up in FLU).
            speed: Max climb rate m/s.
            timeout: Max seconds.
        """
        self._cancel_active()
        loop_fn = loops.make_takeoff_loop(self, altitude, speed)
        self._run_control_loop(loop_fn, timeout, "takeoff")

    def land(
        self,
        speed: float = 0.5,
        timeout: float = 30.0,
    ) -> None:
        """
        Land the drone. Blocks until on the ground.

        Args:
            speed: Max descent rate m/s.
            timeout: Max seconds.
        """
        self._cancel_active()
        loop_fn = loops.make_land_loop(self, speed)
        self._run_control_loop(loop_fn, timeout, "land")

    def hover(self, duration: float = 5.0) -> None:
        """
        Hold current position for duration seconds.

        Args:
            duration: Seconds to hold.
        """
        self._cancel_active()
        loop_fn = loops.make_hover_loop(self, duration)
        self._run_control_loop(loop_fn, duration + 10.0, "hover")

    def fly_to(
        self,
        x: float,
        y: float,
        z: float,
        speed: float = 0.0,
        yaw: float = 0.0,
        timeout: float = 60.0,
    ) -> None:
        """
        Fly to a position in FLU. Z is altitude. Blocks until arrived.

        Args:
            x, y, z: Target position. Z is altitude.
            speed: Approach speed m/s. 0 = use default_speed.
            yaw: Target heading degrees.
            timeout: Max seconds.
        """
        if speed <= 0:
            speed = self.default_speed

        self._cancel_active()

        if self.use_velocity_mode_navigation:
            loop_fn = loops.make_fly_to_velocity_loop(self, x, y, z, speed, yaw_rate=0.0)
        else:
            loop_fn = loops.make_fly_to_loop(self, x, y, z, speed, yaw)

        self._run_control_loop(loop_fn, timeout, "fly_to")

    def fly_path(
        self,
        waypoints: List[Tuple[float, float, float]],
        speed: float = 0.0,
        per_leg_timeout: float = 0.0,
        timeout: float = 0.0,
    ) -> None:
        """
        Follow a sequence of waypoints. Blocks until complete.

        Args:
            waypoints: List of (x, y, z) positions in FLU.
            speed: Max speed m/s. 0 = default.
            per_leg_timeout: Max seconds per waypoint. 0 = default.
            timeout: Max total seconds. 0 = auto-calculated.
        """
        if not waypoints:
            raise ValueError("waypoints list is empty")

        if speed <= 0:
            speed = self.default_speed
        if per_leg_timeout <= 0:
            per_leg_timeout = self.default_leg_timeout
        if timeout <= 0:
            timeout = per_leg_timeout * len(waypoints) * 1.5

        self._cancel_active()
        loop_fn = loops.make_fly_path_loop(self, waypoints, speed, per_leg_timeout)
        self._run_control_loop(loop_fn, timeout, "fly_path")

    def yaw_to(
        self,
        heading_deg: float,
        rate: float = 45.0,
        timeout: float = 10.0,
    ) -> None:
        """
        Rotate to heading while holding position. Blocks until done.

        Args:
            heading_deg: Target yaw degrees.
            rate: Rate hint (for potential fallback).
            timeout: Max seconds.
        """
        self._cancel_active()
        loop_fn = loops.make_yaw_to_loop(self, heading_deg, rate)
        self._run_control_loop(loop_fn, timeout, "yaw_to")

    # ====================================================================
    # High-Level: Async Flight Commands
    # ====================================================================

    def takeoff_async(self, altitude: float = 3.0, speed: float = 1.0, timeout: float = 30.0) -> Future:
        """Non-blocking takeoff. Returns Future."""
        loop_fn = loops.make_takeoff_loop(self, altitude, speed)
        return self._run_async(loop_fn, timeout, "takeoff")

    def land_async(self, speed: float = 0.5, timeout: float = 30.0) -> Future:
        """Non-blocking land. Returns Future."""
        loop_fn = loops.make_land_loop(self, speed)
        return self._run_async(loop_fn, timeout, "land")

    def hover_async(self, duration: float = 5.0) -> Future:
        """Non-blocking hover. Returns Future."""
        loop_fn = loops.make_hover_loop(self, duration)
        return self._run_async(loop_fn, duration + 10.0, "hover")

    def fly_to_async(
        self,
        x: float, y: float, z: float,
        speed: float = 0.0, yaw: float = 0.0,
        timeout: float = 60.0,
    ) -> Future:
        """Non-blocking fly_to. Returns Future."""
        if speed <= 0:
            speed = self.default_speed

        if self.use_velocity_mode_navigation:
            loop_fn = loops.make_fly_to_velocity_loop(self, x, y, z, speed, yaw_rate=0.0)
        else:
            loop_fn = loops.make_fly_to_loop(self, x, y, z, speed, yaw)

        return self._run_async(loop_fn, timeout, "fly_to")

    def fly_path_async(
        self,
        waypoints: List[Tuple[float, float, float]],
        speed: float = 0.0,
        per_leg_timeout: float = 0.0,
        timeout: float = 0.0,
    ) -> Future:
        """Non-blocking fly_path. Returns Future."""
        if not waypoints:
            raise ValueError("waypoints list is empty")

        if speed <= 0:
            speed = self.default_speed
        if per_leg_timeout <= 0:
            per_leg_timeout = self.default_leg_timeout
        if timeout <= 0:
            timeout = per_leg_timeout * len(waypoints) * 1.5

        loop_fn = loops.make_fly_path_loop(self, waypoints, speed, per_leg_timeout)
        return self._run_async(loop_fn, timeout, "fly_path")

    def yaw_to_async(self, heading_deg: float, rate: float = 45.0, timeout: float = 10.0) -> Future:
        """Non-blocking yaw_to. Returns Future."""
        loop_fn = loops.make_yaw_to_loop(self, heading_deg, rate)
        return self._run_async(loop_fn, timeout, "yaw_to")

    # ====================================================================
    # Internal: Control Loop Execution
    # ====================================================================

    def _run_control_loop(
        self,
        loop_fn: Callable[[], bool],
        timeout: float,
        command_name: str,
        future: Optional[Future] = None,
    ) -> None:
        interval = 1.0 / self.control_loop_hz
        deadline = time.monotonic() + timeout

        while True:
            if future is not None and future._is_cancel_requested():
                self._safe_position_hold()
                return

            if time.monotonic() > deadline:
                self._safe_position_hold()
                raise TimeoutError(f"{command_name} timed out after {timeout:.1f}s")

            done = loop_fn()
            if done:
                return

            time.sleep(interval)

    def _run_async(
        self,
        loop_fn: Callable[[], bool],
        timeout: float,
        command_name: str,
    ) -> Future:
        self._cancel_active()

        future = Future()

        def _thread_target() -> None:
            try:
                self._run_control_loop(loop_fn, timeout, command_name, future)
                future._complete()
            except Exception as e:
                future._complete(exception=e)

        with self._command_lock:
            self._active_future = future

        future._start_thread(_thread_target, f"QuadSim-{command_name}")
        return future

    def _cancel_active(self) -> None:
        with self._command_lock:
            future = self._active_future
            self._active_future = None

        if future is not None and not future.done:
            future.cancel()
            future._join(timeout=5.0)

    def _safe_position_hold(self) -> None:
        try:
            sensors = self.get_sensors()
            pos = sensors.gps_position
            self.send_command(x=pos[0], y=pos[1], z=pos[2], w=0.0, mode="position")
        except Exception:
            pass