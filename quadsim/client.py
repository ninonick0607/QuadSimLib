"""
quadsim.client — Python client for the QuadSim drone simulator.

Usage:
    from quadsim import QuadSimClient

    with QuadSimClient("localhost") as client:
        client.set_mode("rate")
        client.send_command(yaw=30, throttle=0.44)
        sensors = client.get_sensors()
        print(f"Attitude: {sensors.imu_attitude}")
"""

import threading
import time
from typing import Optional

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
    High-level Python client for the QuadSim RPC interface.

    Manages a ZMQ REQ socket for commands, an automatic background
    heartbeat thread, and typed response parsing.

    Can be used as a context manager:
        with QuadSimClient("localhost") as client:
            client.set_mode("rate")
            ...

    Or manually:
        client = QuadSimClient("localhost")
        client.connect()
        ...
        client.disconnect()
    """

    # Default ports matching ExternalRpcAdapter
    DEFAULT_COMMAND_PORT = 5555
    DEFAULT_TELEMETRY_PORT = 5556

    def __init__(
        self,
        host: str = "localhost",
        command_port: int = DEFAULT_COMMAND_PORT,
        telemetry_port: int = DEFAULT_TELEMETRY_PORT,
        client_name: str = "QuadSimPython",
        timeout_ms: int = 5000,
        heartbeat_interval: float = 2.0,
    ):
        """
        Args:
            host: QuadSim server hostname or IP.
            command_port: REQ/REP command port (default 5555).
            telemetry_port: PUB telemetry port (default 5556).
            client_name: Name sent to server on connect (appears in logs).
            timeout_ms: ZMQ send/receive timeout in milliseconds.
            heartbeat_interval: Seconds between automatic heartbeat messages.
        """
        self._host = host
        self._command_port = command_port
        self._telemetry_port = telemetry_port
        self._client_name = client_name
        self._timeout_ms = timeout_ms
        self._heartbeat_interval = heartbeat_interval

        # ZMQ state
        self._ctx: Optional[zmq.Context] = None
        self._sock: Optional[zmq.Socket] = None

        # Connection state
        self._connected = False
        self._request_id = 0

        # Thread safety: the REQ socket is not thread-safe, so we serialize
        # all sends behind a lock. The heartbeat thread uses this same lock.
        self._lock = threading.Lock()

        # Heartbeat thread
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._heartbeat_stop = threading.Event()

        # SUB socket for telemetry streaming (Phase 9c)
        self._sub_sock: Optional[zmq.Socket] = None
        self._sub_thread: Optional[threading.Thread] = None
        self._sub_stop = threading.Event()
        self._sub_callback = None

    # ================================================================
    # Properties
    # ================================================================

    @property
    def connected(self) -> bool:
        """True if the client has an active connection to QuadSim."""
        return self._connected

    @property
    def host(self) -> str:
        return self._host

    @property
    def command_port(self) -> int:
        return self._command_port

    # ================================================================
    # Context Manager
    # ================================================================

    def __enter__(self) -> "QuadSimClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()
        return False  # Don't suppress exceptions

    # ================================================================
    # Connection Lifecycle
    # ================================================================

    def connect(self) -> dict:
        """
        Connect to the QuadSim server.

        Creates the ZMQ socket, sends the connect message, and starts
        the background heartbeat thread.

        Returns:
            Server response dict (contains server name, ports).

        Raises:
            ConnectionError: If already connected, or server rejects.
            TimeoutError: If server doesn't respond.
        """
        if self._connected:
            raise ConnectionError("Already connected. Call disconnect() first.")

        # Create fresh ZMQ context and socket
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, self._timeout_ms)
        self._sock.setsockopt(zmq.LINGER, 0)

        address = f"tcp://{self._host}:{self._command_port}"
        self._sock.connect(address)

        # Send connect message
        try:
            resp = self._send("connect", client_name=self._client_name)
        except Exception:
            self._cleanup_socket()
            raise

        self._connected = True
        self._start_heartbeat()

        return resp

    def disconnect(self) -> None:
        """
        Disconnect from the QuadSim server.

        Stops telemetry streaming, heartbeat thread, sends disconnect,
        and closes ZMQ sockets. Safe to call if already disconnected.
        """
        if not self._connected:
            return

        # Stop streaming first (sends unsubscribe to server)
        self.unsubscribe()

        self._stop_heartbeat()

        try:
            self._send("disconnect")
        except Exception:
            pass  # Best-effort — socket might already be dead

        self._connected = False
        self._cleanup_socket()

    # ================================================================
    # Status & Queries
    # ================================================================

    def get_status(self) -> SimStatus:
        """Get simulation-wide status (pause state, authority, timing)."""
        resp = self._send("get_status")
        return SimStatus.from_dict(resp)

    def get_sensors(self) -> SensorData:
        """Get the latest IMU + GPS sensor readings from the selected drone."""
        resp = self._send("get_sensor_data")
        return SensorData.from_dict(resp)

    def get_telemetry(self) -> Telemetry:
        """Get controller state, mode, and motor outputs from the selected drone."""
        resp = self._send("get_telemetry")
        return Telemetry.from_dict(resp)

    # ================================================================
    # Mode & Controller
    # ================================================================

    def set_mode(self, mode: str) -> str:
        """
        Set the flight control mode.

        Args:
            mode: One of "rate", "angle", "velocity", "position",
                  "passthrough", "wrench", "none".

        Returns:
            The confirmed active mode string.
        """
        resp = self._send("set_mode", mode=mode)
        return resp.get("mode", mode)

    def set_controller(self, controller: str) -> str:
        """
        Set the active controller type.

        Args:
            controller: One of "cascade", "geometric".

        Returns:
            The confirmed active controller string.
        """
        resp = self._send("set_controller", controller=controller)
        return resp.get("controller", controller)

    # ================================================================
    # Commands
    # ================================================================

    def send_command(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        w: float = 0.0,
        *,
        # Rate mode aliases
        roll: Optional[float] = None,
        pitch: Optional[float] = None,
        yaw: Optional[float] = None,
        throttle: Optional[float] = None,
        # Velocity mode aliases
        vx: Optional[float] = None,
        vy: Optional[float] = None,
        vz: Optional[float] = None,
        yaw_rate: Optional[float] = None,
        # Optional mode override
        mode: Optional[str] = None,
    ) -> None:
        """
        Send a command to the drone.

        Can use raw Axis4 values (x, y, z, w) or named aliases for clarity.
        Named arguments override positional ones.

        Rate mode:      roll(x), pitch(y), yaw(z) in deg/s, throttle(w) 0-1
        Angle mode:     roll(x), pitch(y) in deg, yaw(z) in deg/s, throttle(w) 0-1
        Velocity mode:  vx(x), vy(y), vz(z) in m/s, yaw_rate(w) in deg/s

        Args:
            x, y, z, w: Raw Axis4 values.
            roll, pitch, yaw, throttle: Rate/angle mode aliases.
            vx, vy, vz, yaw_rate: Velocity mode aliases.
            mode: Optional mode override (sets mode before sending).

        Raises:
            CommandError: If authority is rejected or drone unavailable.
        """
        # Named args override positional
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
            z = yaw_rate

        params = {"x": x, "y": y, "z": z, "w": w}
        if mode is not None:
            params["mode"] = mode

        self._send("send_command", **params)

    # ================================================================
    # Reset Operations
    # ================================================================

    def reset(self) -> None:
        """Full drone reset: physics, controller state, command proxy."""
        self._send("reset_all")

    def reset_pose(
        self,
        x: float = 0.0,
        y: float = 3.0,
        z: float = 0.0,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0,
    ) -> None:
        """Reset the drone to a specific position and orientation."""
        self._send("reset_pose", x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw)

    def reset_rotation(self) -> None:
        """Level the drone (zero roll/pitch, keep yaw) and zero velocities."""
        self._send("reset_rotation")

    def reset_physics(self) -> None:
        """Zero all velocities without moving the drone."""
        self._send("reset_physics")

    def reset_controller(self) -> None:
        """Reset only controller state (PID integrators, allocator)."""
        self._send("reset_controller")

    # ================================================================
    # Simulation Control
    # ================================================================

    def pause(self) -> None:
        """Pause the simulation."""
        self._send("pause")

    def resume(self) -> None:
        """Resume the simulation."""
        self._send("resume")

    def step(self, count: int = 1) -> dict:
        """
        Step the simulation forward while paused.

        Args:
            count: Number of fixed timesteps to advance.

        Returns:
            Dict with sim_time and steps_queued.
        """
        return self._send("step", count=count)

    def set_time_scale(self, scale: float) -> float:
        """
        Set the simulation time scale.

        Args:
            scale: Time multiplier (1.0 = realtime, 2.0 = 2x speed).

        Returns:
            The confirmed time scale.
        """
        resp = self._send("set_time_scale", scale=scale)
        return resp.get("time_scale", scale)

    def reset_simulation(self) -> None:
        """Reset the entire simulation (all drones, time to zero)."""
        self._send("reset_simulation")

    # ================================================================
    # Telemetry Streaming (Phase 9c)
    # ================================================================

    def subscribe(
        self,
        callback,
        topics: Optional[list] = None,
        hz: float = 50.0,
    ) -> None:
        """
        Start receiving telemetry stream from the PUB socket.

        The callback is invoked on a background thread for each received
        message. Messages are parsed into dicts with a "topic" field
        ("sensors" or "telemetry").

        For typed access, use subscribe_sensors() or subscribe_telemetry().

        Args:
            callback: Function called with (dict) for each message.
            topics: List of topics to subscribe to (default: ["sensors", "telemetry"]).
            hz: Requested publish rate in Hz (default: 50).

        Raises:
            ConnectionError: If not connected.
            QuadSimError: If subscription fails.
        """
        if not self._connected:
            raise ConnectionError("Not connected. Call connect() first.")

        if self._sub_thread is not None:
            raise QuadSimError("Already subscribed. Call unsubscribe() first.")

        if topics is None:
            topics = ["sensors", "telemetry"]

        # Tell the server to start publishing
        resp = self._send("subscribe", topics=",".join(topics), hz=hz)
        pub_port = resp.get("telemetry_port", self._telemetry_port)

        # Create SUB socket
        self._sub_sock = self._ctx.socket(zmq.SUB)
        self._sub_sock.setsockopt(zmq.RCVTIMEO, 1000)
        self._sub_sock.setsockopt(zmq.LINGER, 0)
        self._sub_sock.subscribe(b"")  # Subscribe to all messages
        self._sub_sock.connect(f"tcp://{self._host}:{pub_port}")

        # Start receiver thread
        self._sub_callback = callback
        self._sub_stop.clear()
        self._sub_thread = threading.Thread(
            target=self._sub_loop,
            name="QuadSim-Sub",
            daemon=True,
        )
        self._sub_thread.start()

    def subscribe_sensors(self, callback, hz: float = 50.0) -> None:
        """
        Subscribe to sensor data only. Callback receives SensorData objects.

        Args:
            callback: Function called with (SensorData) for each sensor message.
            hz: Requested publish rate in Hz.
        """
        def _typed_callback(data: dict):
            if data.get("topic") == "sensors":
                callback(SensorData.from_dict(data))

        self.subscribe(_typed_callback, topics=["sensors"], hz=hz)

    def subscribe_telemetry(self, callback, hz: float = 50.0) -> None:
        """
        Subscribe to telemetry data only. Callback receives Telemetry objects.

        Args:
            callback: Function called with (Telemetry) for each telemetry message.
            hz: Requested publish rate in Hz.
        """
        def _typed_callback(data: dict):
            if data.get("topic") == "telemetry":
                callback(Telemetry.from_dict(data))

        self.subscribe(_typed_callback, topics=["telemetry"], hz=hz)

    def unsubscribe(self) -> None:
        """Stop receiving telemetry stream and clean up SUB socket."""
        if self._sub_thread is None:
            return

        # Stop the receiver thread
        self._sub_stop.set()
        self._sub_thread.join(timeout=3.0)
        self._sub_thread = None
        self._sub_callback = None

        # Close SUB socket
        if self._sub_sock is not None:
            try:
                self._sub_sock.close()
            except Exception:
                pass
            self._sub_sock = None

        # Tell server to stop publishing
        if self._connected:
            try:
                self._send("unsubscribe")
            except Exception:
                pass

    def _sub_loop(self) -> None:
        """Background thread that receives PUB messages and dispatches to callback."""
        while not self._sub_stop.is_set():
            try:
                raw = self._sub_sock.recv()
                data = msgpack.unpackb(raw, raw=False)
                if self._sub_callback is not None:
                    self._sub_callback(data)
            except zmq.error.Again:
                continue  # Timeout — check stop flag and retry
            except Exception:
                if not self._sub_stop.is_set():
                    continue  # Transient error — keep going
                break

    # ================================================================
    # Core Transport
    # ================================================================

    def _send(self, method: str, **params) -> dict:
        """
        Send an RPC request and return the parsed response.

        This is the single point of contact with the ZMQ socket.
        Thread-safe via self._lock.

        Args:
            method: RPC method name.
            **params: Method parameters.

        Returns:
            Response data dict (status/request_id stripped).

        Raises:
            TimeoutError: Server didn't respond.
            ConnectionError: Not connected (for non-connect methods).
            CommandError: Server returned an error status.
            ProtocolError: Unexpected response format.
        """
        # Allow connect through without connection check
        if method != "connect" and not self._connected:
            raise ConnectionError(f"Not connected. Call connect() first. (method={method})")

        self._request_id += 1
        msg = {"method": method, "request_id": self._request_id, **params}

        with self._lock:
            try:
                self._sock.send(msgpack.packb(msg, use_bin_type=True))
                raw = self._sock.recv()
            except zmq.error.Again:
                raise TimeoutError(
                    f"Server did not respond within {self._timeout_ms}ms "
                    f"(method={method})"
                )
            except zmq.ZMQError as e:
                raise QuadSimError(f"ZMQ error: {e}")

        # Parse response
        try:
            resp = msgpack.unpackb(raw, raw=False)
        except Exception as e:
            raise ProtocolError(f"Failed to decode response: {e}")

        if not isinstance(resp, dict):
            raise ProtocolError(f"Expected dict response, got {type(resp)}")

        # Check status
        status = resp.get("status")
        if status == "error":
            error_msg = resp.get("error_msg", "unknown error")

            # Classify the error
            if error_msg in ("not_connected", "denied", "already_connected"):
                raise ConnectionError(error_msg)
            elif error_msg in ("authority_rejected", "no_drone", "command_failed"):
                raise CommandError(error_msg)
            elif error_msg.startswith("invalid_") or error_msg.startswith("missing_param"):
                raise CommandError(error_msg)
            else:
                raise QuadSimError(error_msg)

        if status != "ok":
            raise ProtocolError(f"Unexpected status: {status}")

        # Strip protocol fields, return data
        resp.pop("status", None)
        resp.pop("request_id", None)
        return resp

    # ================================================================
    # Heartbeat Thread
    # ================================================================

    def _start_heartbeat(self) -> None:
        """Start the background heartbeat thread."""
        self._heartbeat_stop.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="QuadSim-Heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()

    def _stop_heartbeat(self) -> None:
        """Signal the heartbeat thread to stop and wait for it."""
        if self._heartbeat_thread is None:
            return

        self._heartbeat_stop.set()
        self._heartbeat_thread.join(timeout=self._heartbeat_interval + 1.0)
        self._heartbeat_thread = None

    def _heartbeat_loop(self) -> None:
        """Background loop that sends heartbeat messages at regular intervals."""
        while not self._heartbeat_stop.wait(timeout=self._heartbeat_interval):
            if not self._connected:
                break
            try:
                self._send("heartbeat")
            except TimeoutError:
                # Server might be paused or under load — don't crash
                pass
            except Exception:
                # Connection is likely dead
                break

    # ================================================================
    # Socket Cleanup
    # ================================================================

    def _cleanup_socket(self) -> None:
        """Close the ZMQ socket and context."""
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

        if self._ctx is not None:
            try:
                self._ctx.term()
            except Exception:
                pass
            self._ctx = None

    def __del__(self):
        """Ensure cleanup on garbage collection."""
        if self._connected:
            try:
                self.disconnect()
            except Exception:
                pass
        self._cleanup_socket()