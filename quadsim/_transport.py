# quadsim/_transport.py
# Internal Transport Layer
#
# PURPOSE:
#   ZMQ connection management, MessagePack serialization, heartbeat,
#   and PUB/SUB streaming. This is NOT a public API — users never
#   import or interact with this directly.
#
#   QuadSim and Drone both hold a reference to the same Transport
#   instance and route RPC calls through it.

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, List, Optional

import msgpack
import zmq

from .exceptions import (
    CommandError,
    ConnectionError,
    ProtocolError,
    QuadSimError,
    TimeoutError,
)
from .types import SimStatus


class Transport:
    """Internal ZMQ transport. Not part of the public API."""

    def __init__(
        self,
        host: str,
        command_port: int,
        telemetry_port: int,
        client_name: str,
        timeout_ms: int,
        heartbeat_interval: float,
    ):
        self._host = host
        self._command_port = command_port
        self._telemetry_port = telemetry_port
        self._client_name = client_name
        self._timeout_ms = timeout_ms
        self._heartbeat_interval = heartbeat_interval

        self._ctx: Optional[zmq.Context] = None
        self._req: Optional[zmq.Socket] = None
        self._sub: Optional[zmq.Socket] = None

        self._lock = threading.Lock()
        self._connected = False
        self._shutdown = threading.Event()

        self._heartbeat_thread: Optional[threading.Thread] = None
        self._sub_thread: Optional[threading.Thread] = None
        self._stream_callback: Optional[Callable] = None

        self._request_id = 0

    # ====================================================================
    # Connection
    # ====================================================================

    def connect(self) -> SimStatus:
        if self._connected:
            raise ConnectionError("Already connected")

        self._ctx = zmq.Context()
        self._req = self._ctx.socket(zmq.REQ)
        self._req.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        self._req.setsockopt(zmq.SNDTIMEO, self._timeout_ms)
        self._req.setsockopt(zmq.LINGER, 0)
        self._req.connect(f"tcp://{self._host}:{self._command_port}")

        self._shutdown.clear()
        try:
            resp = self._send_locked("connect", {"client_name": self._client_name})
        except Exception as e:
            self._cleanup()
            raise ConnectionError(f"Failed to connect: {e}") from e

        self._connected = True

        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="QuadSim-Heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()

        try:
            return SimStatus.from_dict(self.request("get_status"))
        except Exception:
            return SimStatus.from_dict(resp)

    def disconnect(self) -> None:
        if not self._connected:
            return

        self._stop_streaming()
        self._shutdown.set()

        try:
            with self._lock:
                self._send_unlocked("disconnect", {})
        except Exception:
            pass

        self._connected = False

        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=2.0)
        if self._sub_thread and self._sub_thread.is_alive():
            self._sub_thread.join(timeout=2.0)

        self._cleanup()

    @property
    def connected(self) -> bool:
        return self._connected

    # ====================================================================
    # RPC
    # ====================================================================

    def request(self, method: str, params: Optional[Dict[str, Any]] = None) -> dict:
        """Thread-safe RPC request. Returns the response dict."""
        if not self._connected:
            raise ConnectionError("Not connected to QuadSim server")
        return self._send_locked(method, params or {})

    # ====================================================================
    # Streaming
    # ====================================================================

    def subscribe(
        self,
        callback: Callable,
        topics: Optional[List[str]] = None,
        hz: float = 50.0,
    ) -> None:
        if topics is None:
            topics = ["sensors", "telemetry"]

        self.request("subscribe", {"hz": hz, "topics": ",".join(topics)})

        self._stream_callback = callback

        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.LINGER, 0)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.connect(f"tcp://{self._host}:{self._telemetry_port}")

        self._sub_thread = threading.Thread(
            target=self._sub_loop,
            name="QuadSim-SUB",
            daemon=True,
        )
        self._sub_thread.start()

    def unsubscribe(self) -> None:
        self._stop_streaming()

    # ====================================================================
    # Internals
    # ====================================================================

    def _send_locked(self, method: str, params: dict) -> dict:
        with self._lock:
            return self._send_unlocked(method, params)

    def _send_unlocked(self, method: str, params: dict) -> dict:
        self._request_id += 1
        request = {"method": method, "request_id": self._request_id, **params}

        try:
            self._req.send(msgpack.packb(request, use_bin_type=True))
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
            if error_msg in ("not_connected", "denied"):
                raise ConnectionError(error_msg)
            elif error_msg in ("authority_rejected", "no_drone", "command_failed"):
                raise CommandError(error_msg)
            elif "invalid" in error_msg or "missing" in error_msg:
                raise CommandError(error_msg)
            else:
                raise QuadSimError(error_msg)

        return resp

    def _heartbeat_loop(self) -> None:
        while not self._shutdown.wait(timeout=self._heartbeat_interval):
            if not self._connected:
                break
            try:
                with self._lock:
                    self._send_unlocked("heartbeat", {})
            except Exception:
                break

    def _sub_loop(self) -> None:
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)

        while not self._shutdown.is_set():
            try:
                socks = dict(poller.poll(timeout=100))
            except Exception:
                break

            if self._sub in socks:
                try:
                    raw = self._sub.recv(zmq.NOBLOCK)
                    frame = msgpack.unpackb(raw, raw=False)
                    cb = self._stream_callback
                    if cb is not None:
                        cb(frame.get("topic", ""), frame)
                except Exception:
                    continue

    def _stop_streaming(self) -> None:
        try:
            self.request("unsubscribe")
        except Exception:
            pass

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

    def _cleanup(self) -> None:
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