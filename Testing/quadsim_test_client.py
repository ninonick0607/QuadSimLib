"""
QuadSim Phase 9a — Minimal Python Test Client

Tests the connection lifecycle against the Unity ExternalRpcAdapter.
Validates: connect, heartbeat, get_status, disconnect.

Requirements:
    pip install pyzmq msgpack

Usage:
    1. Start QuadSim in Unity (ExternalRpcAdapter on SimRoot)
    2. Run this script: python quadsim_test_client.py
"""

import time
import zmq
import msgpack


class QuadSimTestClient:
    """Minimal client for testing Phase 9a connection lifecycle."""

    def __init__(self, host: str = "localhost", command_port: int = 5555):
        self.address = f"tcp://{host}:{command_port}"
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.REQ)

        # Set timeouts so we don't hang forever
        self.socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5s receive timeout
        self.socket.setsockopt(zmq.SNDTIMEO, 5000)  # 5s send timeout
        self.socket.setsockopt(zmq.LINGER, 0)  # Don't linger on close

    def _send(self, message: dict) -> dict:
        """Send a MessagePack request and receive the response."""
        packed = msgpack.packb(message, use_bin_type=True)
        self.socket.send(packed)
        reply_bytes = self.socket.recv()
        return msgpack.unpackb(reply_bytes, raw=False)

    def connect_to_sim(self, client_name: str = "test_client") -> dict:
        """Connect ZMQ socket and send connect request."""
        print(f"Connecting to {self.address}...")
        self.socket.connect(self.address)

        response = self._send({
            "method": "connect",
            "client_name": client_name,
        })
        print(f"  Connect response: {response}")
        return response

    def heartbeat(self) -> dict:
        """Send a heartbeat."""
        response = self._send({"method": "heartbeat"})
        print(f"  Heartbeat response: {response}")
        return response

    def get_status(self) -> dict:
        """Get sim status."""
        response = self._send({"method": "get_status"})
        print(f"  Status: {response}")
        return response

    def disconnect(self) -> dict:
        """Send disconnect request."""
        response = self._send({"method": "disconnect"})
        print(f"  Disconnect response: {response}")
        return response

    def close(self):
        """Close the ZMQ socket and context."""
        self.socket.close()
        self.ctx.term()
        print("  ZMQ resources cleaned up.")


def main():
    client = QuadSimTestClient()

    try:
        # --- Step 1: Connect ---
        print("\n=== Step 1: Connect ===")
        resp = client.connect_to_sim("phase9a_test")
        assert resp.get("status") == "ok", f"Connect failed: {resp}"

        # --- Step 2: Get Status ---
        print("\n=== Step 2: Get Status ===")
        resp = client.get_status()
        assert resp.get("status") == "ok", f"Get status failed: {resp}"

        # --- Step 3: Heartbeat Loop ---
        print("\n=== Step 3: Heartbeat (3 beats, 1s apart) ===")
        for i in range(3):
            resp = client.heartbeat()
            assert resp.get("status") == "ok", f"Heartbeat failed: {resp}"
            time.sleep(1.0)

        # --- Step 4: Get Status Again ---
        print("\n=== Step 4: Get Status (post-heartbeat) ===")
        resp = client.get_status()
        assert resp.get("status") == "ok"

        # --- Step 5: Disconnect ---
        print("\n=== Step 5: Disconnect ===")
        resp = client.disconnect()
        assert resp.get("status") == "ok", f"Disconnect failed: {resp}"

        print("\n✅ All Phase 9a tests passed!")

    except zmq.error.Again:
        print("\n❌ Timeout — is QuadSim running with ExternalRpcAdapter?")
    except AssertionError as e:
        print(f"\n❌ Test failed: {e}")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        client.close()


if __name__ == "__main__":
    main()