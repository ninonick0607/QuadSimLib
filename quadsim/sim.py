# quadsim/sim.py
# QuadSim — Simulation Entry Point
#
# PURPOSE:
#   Owns the ZMQ connection and provides sim-level operations.
#   Hands out Drone handles via sim.drone(). The Drone objects
#   route all RPC calls through this connection.
#
# ARCHITECTURE:
#   QuadSim owns:
#     - Connection lifecycle (connect/disconnect/heartbeat)
#     - ZMQ sockets and threading
#     - Sim-level operations (pause/resume/step/time_scale/reset)
#     - Status queries
#
#   Drone owns:
#     - All drone-specific operations (commands, sensors, flight, resets)
#     - Both low-level (send_command, set_mode) and high-level (takeoff, fly_to)
#
# COORDINATE FRAME:
#   Everything external is FLU (X forward, Y left, Z up).
#   The C# adapter handles all frame conversions.

from __future__ import annotations

import atexit
from typing import Optional

from ._transport import Transport
from .drone import Drone
from .types import SimStatus


class QuadSim:
    """
    Simulation entry point. Owns the connection, provides sim-level control.

    Recommended usage::

        with QuadSim() as sim:
            drone = sim.drone()
            drone.takeoff(3.0)
            drone.fly_to(5, 0, 3)
            drone.land()
        # disconnect() called automatically

    Also supports explicit lifecycle::

        sim = QuadSim()
        sim.connect()
        drone = sim.drone()
        # ... do work ...
        sim.disconnect()
    """

    def __init__(
        self,
        host: str = "localhost",
        command_port: int = 5555,
        telemetry_port: int = 5556,
        client_name: str = "QuadSim",
        timeout_ms: int = 5000,
        heartbeat_interval: float = 2.0,
    ):
        self._transport = Transport(
            host=host,
            command_port=command_port,
            telemetry_port=telemetry_port,
            client_name=client_name,
            timeout_ms=timeout_ms,
            heartbeat_interval=heartbeat_interval,
        )
        self._drone: Optional[Drone] = None

        # Step 3: Register atexit handler so even scripts that forget
        # to call disconnect() get a clean shutdown.
        atexit.register(self._atexit_disconnect)

    # ====================================================================
    # Context Manager
    # ====================================================================

    def __enter__(self) -> QuadSim:
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()

    # ====================================================================
    # Connection
    # ====================================================================

    def connect(self) -> SimStatus:
        """Connect to the QuadSim Unity server."""
        return self._transport.connect()

    def disconnect(self) -> None:
        """Disconnect. Cancels any active drone commands first."""
        if self._drone is not None:
            self._drone._cancel_active()
        self._transport.disconnect()

    @property
    def connected(self) -> bool:
        """True if connected to the server."""
        return self._transport.connected

    # ====================================================================
    # Drone Access
    # ====================================================================

    def drone(self, index: int = 0) -> Drone:
        """
        Get a Drone handle.

        Currently single-drone only (index is ignored). The Drone object
        is lightweight — it routes calls through this QuadSim's connection.

        Args:
            index: Drone index (reserved for future multi-drone support).

        Returns:
            Drone handle with full low-level and high-level API.
        """
        if self._drone is None:
            self._drone = Drone(self._transport)
        return self._drone

    # ====================================================================
    # Sim-Level Operations
    # ====================================================================

    def get_status(self) -> SimStatus:
        """Query simulation-wide status."""
        return SimStatus.from_dict(self._transport.request("get_status"))

    def pause(self) -> None:
        """Pause the simulation."""
        self._transport.request("pause")

    def resume(self) -> None:
        """Resume the simulation."""
        self._transport.request("resume")

    def step(self, count: int = 1) -> dict:
        """Advance N physics steps while paused."""
        return self._transport.request("step", {"count": count})

    def set_time_scale(self, scale: float) -> float:
        """Set simulation time scale. Returns confirmed scale."""
        resp = self._transport.request("set_time_scale", {"scale": scale})
        return float(resp.get("time_scale", scale))

    def reset(self) -> None:
        """Reset the entire simulation to initial state."""
        if self._drone is not None:
            self._drone._cancel_active()
        self._transport.request("reset_simulation")

    # ====================================================================
    # Internal: Cleanup
    # ====================================================================

    def _atexit_disconnect(self) -> None:
        """Best-effort disconnect on interpreter shutdown."""
        try:
            if self._transport.connected:
                self.disconnect()
        except Exception:
            pass

    def __del__(self) -> None:
        """Fallback cleanup. Not reliable, but catches some GC cases."""
        try:
            if self._transport.connected:
                self.disconnect()
        except Exception:
            pass