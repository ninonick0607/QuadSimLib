# quadsim/client.py
# Phase 9d: Low-Level RPC Client
#
# PURPOSE:
#   Transport-layer client for QuadSim's ZeroMQ RPC interface.
#   Maps 1:1 to the handler methods in ExternalRpcAdapter.cs.
#   Handles connection lifecycle, heartbeat, request/response,
#   and PUB/SUB telemetry streaming.
#
# THREADING MODEL:
#   - REQ socket: guarded by threading.Lock, shared between user thread
#     and heartbeat thread
#   - Heartbeat thread: daemon, sends heartbeat every N seconds
#   - SUB thread: daemon, receives PUB messages, dispatches to user callback
#   - All three threads coordinate via threading.Event for clean shutdown
#
# DEPENDENCIES:
#   pyzmq, msgpack

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, List, Optional, Union

import msgpack
import zmq

from .exceptions import (
    CommandError,
    ConnectionError,
    ProtocolError,
    QuadSimError,
    TimeoutError,
)
from .types import SensorData, SimStatus, Telemetry


class QuadSimClient:
    """
    Low-level RPC client for QuadSim.

    Provides direct access to all 16+ RPC methods exposed by
    ExternalRpcAdapter. This is the transport layer — it handles
    ZeroMQ sockets, MessagePack serialization, heartbeat keepalive,
    and PUB/SUB streaming.

    For high-level intent-based control (takeoff, fly_to, land),
    use QuadSimApi instead, which wraps this client.

    Usage::

        from quadsim import QuadSimClient

        with QuadSimClient() as client:
            client.set_mode("rate")
            client.send_command(roll=0, pitch=0, yaw=5, throttle=0.44)
            sensors = client.get_sensors()
            print(sensors.gps_position)
    """

    def __init__(
        self,
        host: str = "localhost",
        command_port: int = 5555,
        telemetry_port: int = 5556,
        client_name: str = "PythonClient",
        timeout_ms: int = 5000,
        heartbeat_interval: float = 2.0,
    ):
        self._host = host
        self._command_port = command_port
        self._telemetry_port = telemetry_port
        self._client_name = client_name
        self._timeout_ms = timeout_ms
        self._heartbeat_interval = heartbeat_interval

        # ZMQ context and sockets (created on connect)
        self._ctx: Optional[zmq.Context] = None
        self._req: Optional[zmq.Socket] = None
        self._sub: Optional[zmq.Socket] = None

        # Thread coordination
        self._lock = threading.Lock()          # Protects REQ socket
        self._connected = False
        self._shutdown = threading.Event()

        # Heartbeat
        self._heartbeat_thread: Optional[threading.Thread] = None

        # Streaming
        self._sub_thread: Optional[threading.Thread] = None
        self._stream_callback: Optional[Callable] = None
        self._stream_topics: List[str] = []

        # Request ID counter
        self._request_id = 0

    # ========================================================================
    # Context Manager
    # ========================================================================

    def __enter__(self) -> QuadSimClient:
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()

    # ========================================================================
    # Connection Lifecycle
    # ========================================================================

    def connect(self) -> SimStatus:
        """
        Connect to the QuadSim server.

        Creates ZMQ sockets, sends the 'connect' RPC, and starts
        the heartbeat keepalive thread.

        Returns:
            SimStatus with initial server state.

        Raises:
            ConnectionError: If the server denies the connection.
            TimeoutError: If the server doesn't respond.
        """
        if self._connected:
            raise ConnectionError("Already connected")

        self._ctx = zmq.Context()

        # REQ socket for commands
        self._req = self._ctx.socket(zmq.REQ)
        self._req.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        self._req.setsockopt(zmq.SNDTIMEO, self._timeout_ms)
        self._req.setsockopt(zmq.LINGER, 0)
        self._req.connect(f"tcp://{self._host}:{self._command_port}")

        # Send connect request
        self._shutdown.clear()
        try:
            resp = self._send_raw("connect", {"client_name": self._client_name})
        except Exception as e:
            self._cleanup_sockets()
            raise ConnectionError(f"Failed to connect: {e}") from e

        self._connected = True

        # Start heartbeat thread
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="QuadSim-Heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()

        # Query initial status
        try:
            return self.get_status()
        except Exception:
            # Connection succeeded even if status query fails
            return SimStatus.from_dict(resp)

    def disconnect(self) -> None:
        """
        Disconnect from the QuadSim server.

        Stops streaming, sends 'disconnect' RPC, stops heartbeat,
        and cleans up ZMQ sockets. Idempotent — safe to call multiple times.
        """
        if not self._connected:
            return

        # Stop streaming first
        try:
            self.unsubscribe()
        except Exception:
            pass

        # Signal threads to stop
        self._shutdown.set()

        # Send disconnect (best-effort)
        try:
            with self._lock:
                self._send_raw_unlocked("disconnect", {})
        except Exception:
            pass

        self._connected = False

        # Wait for threads
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=2.0)
        if self._sub_thread and self._sub_thread.is_alive():
            self._sub_thread.join(timeout=2.0)

        self._cleanup_sockets()

    @property
    def connected(self) -> bool:
        """True if the client believes it is connected to the server."""
        return self._connected

    # ========================================================================
    # Status & Queries
    # ========================================================================

    def get_status(self) -> SimStatus:
        """Query simulation-wide status."""
        resp = self._request("get_status")
        return SimStatus.from_dict(resp)

    def get_sensors(self) -> SensorData:
        """Query the selected drone's sensor readings."""
        resp = self._request("get_sensor_data")
        return SensorData.from_dict(resp)

    def get_telemetry(self) -> Telemetry:
        """Query the selected drone's operational telemetry."""
        resp = self._request("get_telemetry")
        return Telemetry.from_dict(resp)

    # ========================================================================
    # Mode & Controller
    # ========================================================================

    def set_mode(self, mode: str) -> str:
        """
        Set the drone's goal mode.

        Args:
            mode: One of "none", "rate", "angle", "velocity", "position",
                  "passthrough", "wrench".

        Returns:
            The mode string as confirmed by the server.
        """
        resp = self._request("set_mode", {"mode": mode})
        return resp.get("mode", mode)

    def set_controller(self, controller: str) -> str:
        """
        Set the drone's active controller type.

        Args:
            controller: One of "cascade", "geometric".

        Returns:
            The controller string as confirmed by the server.
        """
        resp = self._request("set_controller", {"controller": controller})
        return resp.get("controller", controller)

    # ========================================================================
    # Commands
    # ========================================================================

    def send_command(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        w: float = 0.0,
        *,
        # Named aliases for readability
        roll: Optional[float] = None,
        pitch: Optional[float] = None,
        yaw: Optional[float] = None,
        throttle: Optional[float] = None,
        vx: Optional[float] = None,
        vy: Optional[float] = None,
        vz: Optional[float] = None,
        yaw_rate: Optional[float] = None,
        mode: Optional[str] = None,
    ) -> None:
        """
        Send a raw Axis4 command to the drone.

        Can be called with positional args (x, y, z, w) or with named
        keyword aliases for the active mode:

        Rate/Angle mode::
            send_command(roll=0, pitch=0, yaw=30, throttle=0.44)

        Velocity mode::
            send_command(vx=1.0, vy=0, vz=0, yaw_rate=0)

        Position mode::
            send_command(x=5, y=3, z=0, w=0)

        With mode override::
            send_command(vx=1.0, vy=0, vz=0.5, yaw_rate=0, mode="velocity")

        Raises:
            CommandError: If authority is rejected or command fails.
        """
        # Resolve named aliases → x, y, z, w
        if roll is not None:
            x = roll
        if pitch is not None:
            y = pitch
        if yaw is not None:
            z = yaw
        if throttle is not None:
            w = throttle
        if vx is not None:
            x = vx
        if vy is not None:
            y = vy
        if vz is not None:
            z = vz
        if yaw_rate is not None:
            w = yaw_rate

        params: Dict[str, Any] = {"x": x, "y": y, "z": z, "w": w}
        if mode is not None:
            params["mode"] = mode

        self._request("send_command", params)

    # ========================================================================
    # Resets
    # ========================================================================

    def reset(self) -> None:
        """Full drone reset (pose + physics + controller)."""
        self._request("reset_all")

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
        """Reset drone position and rotation to specified pose."""
        self._request("reset_pose", {
            "x": x, "y": y, "z": z,
            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
        })

    def reset_rotation(self) -> None:
        """Reset drone rotation to level (identity or spawn rotation)."""
        self._request("reset_rotation")

    def reset_physics(self) -> None:
        """Zero out all velocities and angular velocities."""
        self._request("reset_physics")

    def reset_controller(self) -> None:
        """Reset PID integrators and controller internal state."""
        self._request("reset_controller")

    # ========================================================================
    # Time Control
    # ========================================================================

    def pause(self) -> None:
        """Pause the simulation."""
        self._request("pause")

    def resume(self) -> None:
        """Resume the simulation."""
        self._request("resume")

    def step(self, count: int = 1) -> dict:
        """
        Advance the simulation by N physics steps (while paused).

        Args:
            count: Number of fixed-timestep steps to advance.

        Returns:
            Dict with step result data from the server.
        """
        return self._request("step", {"count": count})

    def set_time_scale(self, scale: float) -> float:
        """
        Set the simulation time scale.

        Args:
            scale: Time multiplier (1.0 = real-time, 2.0 = double speed).

        Returns:
            The time scale as confirmed by the server.
        """
        resp = self._request("set_time_scale", {"scale": scale})
        return float(resp.get("time_scale", scale))

    def reset_simulation(self) -> None:
        """Reset the entire simulation to initial state."""
        self._request("reset_simulation")

    # ========================================================================
    # Telemetry Streaming (PUB/SUB)
    # ========================================================================

    def subscribe(
        self,
        callback: Callable[[str, dict], None],
        topics: Optional[List[str]] = None,
        hz: float = 50.0,
    ) -> None:
        """
        Subscribe to streaming telemetry.

        Args:
            callback: Called with (topic, data_dict) for each frame.
            topics: List of topic strings, e.g. ["sensors", "telemetry"].
                    Defaults to both.
            hz: Requested publish rate in Hz.
        """
        if topics is None:
            topics = ["sensors", "telemetry"]

        # Send subscribe RPC
        self._request("subscribe", {
            "hz": hz,
            "topics": ",".join(topics),
        })

        self._stream_callback = callback
        self._stream_topics = topics

        # Create SUB socket
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.LINGER, 0)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")  # Accept all topics
        self._sub.connect(f"tcp://{self._host}:{self._telemetry_port}")

        # Start receiver thread
        self._sub_thread = threading.Thread(
            target=self._sub_loop,
            name="QuadSim-SUB",
            daemon=True,
        )
        self._sub_thread.start()

    def subscribe_sensors(
        self,
        callback: Callable[[SensorData], None],
        hz: float = 50.0,
    ) -> None:
        """
        Subscribe to sensor data only, with typed callback.

        Args:
            callback: Called with a SensorData instance for each frame.
            hz: Requested publish rate.
        """
        def _typed_callback(topic: str, data: dict) -> None:
            if topic == "sensors":
                callback(SensorData.from_dict(data))

        self.subscribe(_typed_callback, topics=["sensors"], hz=hz)

    def subscribe_telemetry(
        self,
        callback: Callable[[Telemetry], None],
        hz: float = 50.0,
    ) -> None:
        """
        Subscribe to telemetry data only, with typed callback.

        Args:
            callback: Called with a Telemetry instance for each frame.
            hz: Requested publish rate.
        """
        def _typed_callback(topic: str, data: dict) -> None:
            if topic == "telemetry":
                callback(Telemetry.from_dict(data))

        self.subscribe(_typed_callback, topics=["telemetry"], hz=hz)

    def unsubscribe(self) -> None:
        """Stop streaming telemetry. Idempotent."""
        if self._sub is None:
            return

        # Send unsubscribe RPC (best-effort)
        try:
            self._request("unsubscribe")
        except Exception:
            pass

        # Clean up SUB socket
        self._stream_callback = None
        if self._sub is not None:
            try:
                self._sub.close()
            except Exception:
                pass
            self._sub = None

        if self._sub_thread and self._sub_thread.is_alive():
            self._sub_thread.join(timeout=2.0)
        self._sub_thread = None

    # ========================================================================
    # Internal: Request/Response
    # ========================================================================

    def _request(self, method: str, params: Optional[Dict[str, Any]] = None) -> dict:
        """
        Send an RPC request and return the response data dict.

        Thread-safe: acquires _lock before touching the REQ socket.

        Raises:
            ConnectionError: If not connected.
            TimeoutError: If the server doesn't respond within timeout_ms.
            CommandError: If the server returns an error status.
            ProtocolError: If the response can't be parsed.
        """
        if not self._connected:
            raise ConnectionError("Not connected to QuadSim server")

        with self._lock:
            return self._send_raw_unlocked(method, params or {})

    def _send_raw(self, method: str, params: dict) -> dict:
        """Send request with lock acquisition. Used during connect()."""
        with self._lock:
            return self._send_raw_unlocked(method, params)

    def _send_raw_unlocked(self, method: str, params: dict) -> dict:
        """
        Send a request and wait for the response. Caller must hold _lock.

        Returns the response data dict (everything except status/request_id).
        """
        self._request_id += 1
        request = {
            "method": method,
            "request_id": self._request_id,
            **params,
        }

        try:
            raw = msgpack.packb(request, use_bin_type=True)
            self._req.send(raw)
        except zmq.Again:
            raise TimeoutError(f"Send timed out for '{method}'")
        except Exception as e:
            raise ProtocolError(f"Failed to send '{method}': {e}") from e

        try:
            raw_resp = self._req.recv()
        except zmq.Again:
            raise TimeoutError(f"Recv timed out for '{method}'")
        except Exception as e:
            raise ProtocolError(f"Failed to recv '{method}': {e}") from e

        try:
            resp = msgpack.unpackb(raw_resp, raw=False)
        except Exception as e:
            raise ProtocolError(f"Failed to decode response for '{method}': {e}") from e

        if not isinstance(resp, dict):
            raise ProtocolError(f"Response is not a dict for '{method}'")

        status = resp.get("status", "")
        if status == "error":
            error_msg = resp.get("error_msg", "unknown error")
            # Map known error codes to specific exception types
            if error_msg in ("not_connected", "denied"):
                raise ConnectionError(error_msg)
            elif error_msg in ("authority_rejected", "no_drone", "command_failed"):
                raise CommandError(error_msg)
            elif "invalid" in error_msg or "missing" in error_msg:
                raise CommandError(error_msg)
            else:
                raise QuadSimError(error_msg)

        # Return the full dict — callers extract what they need
        return resp

    # ========================================================================
    # Internal: Heartbeat Thread
    # ========================================================================

    def _heartbeat_loop(self) -> None:
        """Daemon thread: send heartbeat at regular intervals."""
        while not self._shutdown.wait(timeout=self._heartbeat_interval):
            if not self._connected:
                break
            try:
                with self._lock:
                    self._send_raw_unlocked("heartbeat", {})
            except Exception:
                # Heartbeat failure — don't crash the thread, just stop
                break

    # ========================================================================
    # Internal: SUB Receiver Thread
    # ========================================================================

    def _sub_loop(self) -> None:
        """Daemon thread: receive PUB messages and dispatch to callback."""
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)

        while not self._shutdown.is_set():
            try:
                socks = dict(poller.poll(timeout=100))  # 100ms poll
            except Exception:
                break

            if self._sub in socks:
                try:
                    raw = self._sub.recv(zmq.NOBLOCK)
                    frame = msgpack.unpackb(raw, raw=False)
                    topic = frame.get("topic", "")
                    callback = self._stream_callback
                    if callback is not None:
                        callback(topic, frame)
                except Exception:
                    continue  # Don't crash on bad frames

    # ========================================================================
    # Internal: Socket Cleanup
    # ========================================================================

    def _cleanup_sockets(self) -> None:
        """Close all ZMQ sockets and destroy the context."""
        if self._sub is not None:
            try:
                self._sub.close()
            except Exception:
                pass
            self._sub = None

        if self._req is not None:
            try:
                self._req.close()
            except Exception:
                pass
            self._req = None

        if self._ctx is not None:
            try:
                self._ctx.term()
            except Exception:
                pass
            self._ctx = None