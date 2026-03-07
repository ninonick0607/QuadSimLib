# quadsim/api.py
# Phase 10: High-Level Python SDK
#
# PURPOSE:
#   AirSim-style high-level API for QuadSim. Researchers interact with
#   intent-based methods (takeoff, fly_to, hover, land) instead of raw
#   Axis4 commands. The SDK handles mode switching, control loops, and
#   blocking/async patterns internally.
#
# RELATIONSHIP TO QuadSimClient:
#   QuadSimApi WRAPS QuadSimClient — it does NOT replace it.
#   The low-level client is always accessible at `api.client` for
#   power users who need raw control loop access.
#
#   ┌──────────────────────────────────────────┐
#   │  QuadSimApi (this file — HIGH LEVEL)     │  ← Researchers use this
#   │  takeoff(), fly_to(), hover(), land()    │
#   ├──────────────────────────────────────────┤
#   │  QuadSimClient (client.py — TRANSPORT)   │  ← Power users can use directly
#   │  send_command(), get_sensors(), etc.      │
#   └──────────────────────────────────────────┘
#
# THREADING:
#   - Blocking methods run on the caller's thread.
#   - Async methods spin up a daemon thread per command.
#   - Only one command can be active at a time. Starting a new async
#     command cancels the in-progress one and waits for its thread
#     to exit before proceeding.
#   - The client._lock protects the REQ socket across all threads.
#
# COORDINATE FRAME:
#   Same as the simulation: Unity world coordinates, Y-up.
#   fly_to(x, y, z) uses the same frame as get_sensors().gps_position.

from __future__ import annotations

import threading
import time
from typing import Callable, List, Optional, Tuple

from .client import QuadSimClient
from .exceptions import QuadSimError, TimeoutError
from .future import Future
from .types import SensorData, SimStatus, Telemetry
from . import _control_loops as loops


class QuadSimApi:
    """
    High-level flight API for QuadSim.

    Provides intent-based methods like ``takeoff()``, ``fly_to()``,
    ``hover()``, and ``land()`` that handle mode switching and control
    loops internally.

    The underlying ``QuadSimClient`` is always accessible at ``api.client``
    for raw RPC access.

    Usage::

        from quadsim import QuadSimApi

        with QuadSimApi() as api:
            api.takeoff(altitude=3.0)
            api.fly_to(10, 3, 0)
            api.hover(duration=5.0)
            api.land()

    Async usage::

        api = QuadSimApi()
        api.connect()

        future = api.fly_to_async(10, 3, 0)
        while not future.done:
            print(f"Position: {api.get_position()}")
            time.sleep(0.1)

        api.disconnect()
    """

    def __init__(
        self,
        host: str = "localhost",
        command_port: int = 5555,
        telemetry_port: int = 5556,
        client_name: str = "QuadSimApi",
        timeout_ms: int = 5000,
        heartbeat_interval: float = 2.0,
    ):
        # ── Low-level client (public — power users can access directly) ──
        self.client = QuadSimClient(
            host=host,
            command_port=command_port,
            telemetry_port=telemetry_port,
            client_name=client_name,
            timeout_ms=timeout_ms,
            heartbeat_interval=heartbeat_interval,
        )

        # ── Tuning parameters (public — user-configurable) ──

        #: Maximum vertical speed for takeoff/land (m/s).
        self.hover_throttle: float = 0.44

        #: 3D distance threshold for "arrived at position" (meters).
        self.position_tolerance: float = 0.5

        #: Vertical distance threshold for "at target altitude" (meters).
        self.altitude_tolerance: float = 0.3

        #: Altitude below which the drone is considered "on the ground" (meters).
        self.landing_altitude: float = 0.15

        #: Default speed for fly_to if not specified (m/s).
        self.default_speed: float = 2.0

        #: Control loop frequency for high-level commands (Hz).
        self.control_loop_hz: float = 50.0

        #: Default per-leg timeout for fly_path waypoints (seconds).
        self.default_leg_timeout: float = 30.0

        #: If True, use velocity-mode fallback for fly_to instead of position mode.
        #: Set this to True if the position controller isn't tuned.
        self.use_velocity_mode_navigation: bool = False

        # ── Internal state ──
        self._active_future: Optional[Future] = None
        self._command_lock = threading.Lock()  # Protects _active_future

    # ====================================================================
    # Context Manager
    # ====================================================================

    def __enter__(self) -> QuadSimApi:
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()

    # ====================================================================
    # Connection Lifecycle
    # ====================================================================

    def connect(self) -> SimStatus:
        """
        Connect to the QuadSim server.

        Delegates to ``QuadSimClient.connect()``.

        Returns:
            SimStatus with initial server state.
        """
        return self.client.connect()

    def disconnect(self) -> None:
        """
        Disconnect from the QuadSim server.

        Cancels any active async command first, then delegates
        to ``QuadSimClient.disconnect()``.
        """
        self._cancel_active()
        self.client.disconnect()

    @property
    def connected(self) -> bool:
        """True if connected to the server."""
        return self.client.connected

    # ====================================================================
    # Convenience Queries (thin wrappers)
    # ====================================================================

    def get_position(self) -> Tuple[float, float, float]:
        """
        Get the drone's current GPS position in Unity world coordinates.

        Returns:
            (x, y, z) where Y is altitude.
        """
        return self.client.get_sensors().gps_position

    def get_attitude(self) -> Tuple[float, float, float]:
        """
        Get the drone's current attitude.

        Returns:
            (roll, pitch, yaw) in degrees, FLU convention.
        """
        return self.client.get_sensors().imu_attitude

    def get_velocity(self) -> Tuple[float, float, float]:
        """
        Get the drone's current velocity.

        Returns:
            (vx, vy, vz) in m/s, FLU convention.
        """
        return self.client.get_sensors().imu_vel

    def get_sensors(self) -> SensorData:
        """Get full sensor data snapshot. Passthrough to client."""
        return self.client.get_sensors()

    def get_telemetry(self) -> Telemetry:
        """Get operational telemetry snapshot. Passthrough to client."""
        return self.client.get_telemetry()

    def get_status(self) -> SimStatus:
        """Get simulation status. Passthrough to client."""
        return self.client.get_status()

    def is_airborne(self) -> bool:
        """True if the drone's altitude is above the landing threshold."""
        pos = self.get_position()
        return pos[1] > self.landing_altitude

    # ====================================================================
    # Blocking Flight Commands
    # ====================================================================

    def takeoff(
        self,
        altitude: float = 3.0,
        speed: float = 1.0,
        timeout: float = 30.0,
    ) -> None:
        """
        Take off to the specified altitude and stabilize.

        Blocks until the drone reaches the target altitude (within
        ``altitude_tolerance``) or the timeout expires.

        Args:
            altitude: Target altitude in meters (Y-up).
            speed: Maximum climb rate in m/s.
            timeout: Maximum seconds to wait.

        Raises:
            TimeoutError: If the drone doesn't reach altitude in time.
            CommandError: If authority is rejected.
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
        Land the drone.

        Descends at the specified speed, slowing near the ground for
        a soft touchdown. Blocks until the drone is on the ground
        (altitude < ``landing_altitude``).

        Args:
            speed: Maximum descent rate in m/s.
            timeout: Maximum seconds to wait.

        Raises:
            TimeoutError: If landing doesn't complete in time.
        """
        self._cancel_active()
        loop_fn = loops.make_land_loop(self, speed)
        self._run_control_loop(loop_fn, timeout, "land")

    def hover(self, duration: float = 5.0) -> None:
        """
        Hold current position for the specified duration.

        Captures the drone's current position, switches to position mode,
        and holds for ``duration`` seconds (wall-clock time).

        Args:
            duration: Seconds to hold position.
        """
        self._cancel_active()
        loop_fn = loops.make_hover_loop(self, duration)
        # Timeout is duration + generous buffer for mode switching
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
        Fly to a target position in Unity world coordinates.

        Blocks until the drone is within ``position_tolerance`` of the
        target or the timeout expires.

        Args:
            x, y, z: Target position (Y is altitude).
            speed: Desired approach speed in m/s. 0 = use default_speed.
            yaw: Target yaw heading in degrees.
            timeout: Maximum seconds to wait.

        Raises:
            TimeoutError: If the drone doesn't arrive in time.
            CommandError: If authority is rejected.
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
        Follow a sequence of waypoints.

        Each waypoint is (x, y, z) in Unity world coordinates.
        The drone flies to each waypoint in order. Per-leg timeouts
        catch stalls at individual waypoints.

        Args:
            waypoints: List of (x, y, z) positions.
            speed: Max speed in m/s. 0 = use default_speed.
            per_leg_timeout: Max seconds per waypoint. 0 = use default_leg_timeout.
            timeout: Max total seconds. 0 = per_leg_timeout * len(waypoints) * 1.5.

        Raises:
            TimeoutError: If any leg or the total path times out.
            ValueError: If waypoints is empty.
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
        Rotate to an absolute heading while holding position.

        Args:
            heading_deg: Target yaw in degrees.
            rate: Yaw rate hint (used in potential rate-mode fallback).
            timeout: Maximum seconds to wait.

        Raises:
            TimeoutError: If the heading isn't reached in time.
        """
        self._cancel_active()
        loop_fn = loops.make_yaw_to_loop(self, heading_deg, rate)
        self._run_control_loop(loop_fn, timeout, "yaw_to")

    # ====================================================================
    # Async Flight Commands
    # ====================================================================

    def takeoff_async(
        self,
        altitude: float = 3.0,
        speed: float = 1.0,
        timeout: float = 30.0,
    ) -> Future:
        """
        Non-blocking takeoff. Returns a Future immediately.

        See ``takeoff()`` for parameter docs.
        """
        loop_fn = loops.make_takeoff_loop(self, altitude, speed)
        return self._run_async(loop_fn, timeout, "takeoff")

    def land_async(
        self,
        speed: float = 0.5,
        timeout: float = 30.0,
    ) -> Future:
        """Non-blocking landing. Returns a Future immediately."""
        loop_fn = loops.make_land_loop(self, speed)
        return self._run_async(loop_fn, timeout, "land")

    def hover_async(self, duration: float = 5.0) -> Future:
        """Non-blocking position hold. Returns a Future immediately."""
        loop_fn = loops.make_hover_loop(self, duration)
        return self._run_async(loop_fn, duration + 10.0, "hover")

    def fly_to_async(
        self,
        x: float,
        y: float,
        z: float,
        speed: float = 0.0,
        yaw: float = 0.0,
        timeout: float = 60.0,
    ) -> Future:
        """Non-blocking fly_to. Returns a Future immediately."""
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
        """Non-blocking fly_path. Returns a Future immediately."""
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

    def yaw_to_async(
        self,
        heading_deg: float,
        rate: float = 45.0,
        timeout: float = 10.0,
    ) -> Future:
        """Non-blocking yaw_to. Returns a Future immediately."""
        loop_fn = loops.make_yaw_to_loop(self, heading_deg, rate)
        return self._run_async(loop_fn, timeout, "yaw_to")

    # ====================================================================
    # Sim Control Passthroughs
    # ====================================================================

    def pause(self) -> None:
        """Pause the simulation."""
        self.client.pause()

    def resume(self) -> None:
        """Resume the simulation."""
        self.client.resume()

    def step(self, count: int = 1) -> dict:
        """Advance the simulation by N physics steps (while paused)."""
        return self.client.step(count)

    def set_time_scale(self, scale: float) -> float:
        """Set the simulation time scale. Returns confirmed scale."""
        return self.client.set_time_scale(scale)

    def reset(self) -> None:
        """Full drone reset (pose + physics + controller)."""
        self._cancel_active()
        self.client.reset()

    def reset_simulation(self) -> None:
        """Reset the entire simulation to initial state."""
        self._cancel_active()
        self.client.reset_simulation()

    # ====================================================================
    # Internal: Control Loop Runner
    # ====================================================================

    def _run_control_loop(
        self,
        loop_fn: Callable[[], bool],
        timeout: float,
        command_name: str,
        future: Optional[Future] = None,
    ) -> None:
        """
        Shared loop runner for all blocking commands.

        Calls loop_fn() at control_loop_hz until it returns True or
        the timeout expires. If a Future is provided, checks for
        cancellation each tick.

        Args:
            loop_fn: The tick function. Returns True when done.
            timeout: Max seconds before raising TimeoutError.
            command_name: For error messages.
            future: Optional Future for async cancellation support.

        Raises:
            TimeoutError: If timeout expires.
            QuadSimError: If the loop_fn raises.
        """
        interval = 1.0 / self.control_loop_hz
        deadline = time.monotonic() + timeout

        while True:
            # Check cancellation (async only)
            if future is not None and future._is_cancel_requested():
                # Switch to position hold at current location for safety
                self._safe_position_hold()
                return

            # Check timeout
            if time.monotonic() > deadline:
                # Try to hold position before raising
                self._safe_position_hold()
                raise TimeoutError(
                    f"{command_name} timed out after {timeout:.1f}s"
                )

            # Run one tick
            done = loop_fn()
            if done:
                return

            time.sleep(interval)

    def _safe_position_hold(self) -> None:
        """
        Best-effort switch to position hold at current location.

        Called when a command is cancelled or times out. Prevents the
        drone from drifting or falling after interruption.
        """
        try:
            sensors = self.client.get_sensors()
            pos = sensors.gps_position
            self.client.send_command(
                x=pos[0], y=pos[1], z=pos[2], w=0.0,
                mode="position",
            )
        except Exception:
            # If we can't hold position (e.g., disconnected), that's okay.
            # The caller will handle the disconnect separately.
            pass

    # ====================================================================
    # Internal: Async Command Execution
    # ====================================================================

    def _run_async(
        self,
        loop_fn: Callable[[], bool],
        timeout: float,
        command_name: str,
    ) -> Future:
        """
        Run a control loop on a background thread.

        Cancels any in-progress async command first, then starts a new
        daemon thread that runs _run_control_loop with the given Future.

        Args:
            loop_fn: The tick function.
            timeout: Max seconds.
            command_name: For thread naming and error messages.

        Returns:
            A Future that can be polled, waited on, or cancelled.
        """
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
        """
        Cancel the currently active async command, if any.

        Signals cancellation and waits for the thread to exit before
        returning. This prevents two threads from fighting over
        send_command.
        """
        with self._command_lock:
            future = self._active_future
            self._active_future = None

        if future is not None and not future.done:
            future.cancel()
            future._join(timeout=5.0)