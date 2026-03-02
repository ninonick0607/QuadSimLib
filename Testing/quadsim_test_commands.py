#!/usr/bin/env python3
"""
quadsim_test_commands.py — Phase 9b End-to-End Test Script

Tests all Phase 9b RPC methods against a running QuadSim instance.
Prerequisites:
  - QuadSim running in Unity with ExternalRpcAdapter on SimRoot
  - ControlAuthorityManager default set to External (or auto-accept enabled)
  - pip install pyzmq msgpack

Usage:
  python quadsim_test_commands.py [--host localhost] [--port 5555]
"""

import sys
import time
import argparse
import zmq
import msgpack


# ============================================================================
# Test Helpers
# ============================================================================

class QuadSimTestClient:
    """Minimal test client for Phase 9b validation."""

    def __init__(self, host: str = "localhost", port: int = 5555, timeout_ms: int = 5000):
        self.address = f"tcp://{host}:{port}"
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self.sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self.sock.setsockopt(zmq.LINGER, 0)
        self._request_id = 0

    def connect_socket(self):
        print(f"  Connecting ZMQ socket to {self.address}...")
        self.sock.connect(self.address)

    def close(self):
        self.sock.close()
        self.ctx.term()

    def send(self, method: str, **params) -> dict:
        """Send an RPC request and return the parsed response."""
        self._request_id += 1
        msg = {"method": method, "request_id": self._request_id, **params}
        self.sock.send(msgpack.packb(msg, use_bin_type=True))
        raw = self.sock.recv()
        resp = msgpack.unpackb(raw, raw=False)
        return resp

    def assert_ok(self, resp: dict, context: str = ""):
        """Assert the response status is 'ok'."""
        if resp.get("status") != "ok":
            print(f"  ❌ FAIL {context}: Expected status=ok, got: {resp}")
            sys.exit(1)
        return resp

    def assert_error(self, resp: dict, expected_msg: str = None, context: str = ""):
        """Assert the response status is 'error'."""
        if resp.get("status") != "error":
            print(f"  ❌ FAIL {context}: Expected status=error, got: {resp}")
            sys.exit(1)
        if expected_msg and resp.get("error_msg") != expected_msg:
            print(f"  ❌ FAIL {context}: Expected error_msg='{expected_msg}', "
                  f"got: '{resp.get('error_msg')}'")
            sys.exit(1)
        return resp


# ============================================================================
# Test Steps
# ============================================================================

def test_connection(c: QuadSimTestClient):
    """Step 1: Connect and verify."""
    print("\n=== Step 1: Connect ===")
    resp = c.assert_ok(c.send("connect", client_name="phase9b_test"), "connect")
    print(f"  ✅ Connected: server={resp.get('server')}, "
          f"ports={resp.get('command_port')}/{resp.get('telemetry_port')}")


def test_get_status(c: QuadSimTestClient):
    """Step 2: Get status and verify authority is External."""
    print("\n=== Step 2: Get Status ===")
    resp = c.assert_ok(c.send("get_status"), "get_status")
    assert resp.get("client_connected") is True, "Expected client_connected=True"
    assert resp.get("authority") == "External", f"Expected authority=External, got {resp.get('authority')}"
    print(f"  ✅ Status: authority={resp['authority']}, paused={resp['is_paused']}, "
          f"sim_time={resp['sim_time']:.3f}")


def test_set_mode(c: QuadSimTestClient):
    """Step 3: Set mode to Rate."""
    print("\n=== Step 3: Set Mode → Rate ===")
    resp = c.assert_ok(c.send("set_mode", mode="rate"), "set_mode(rate)")
    assert resp.get("mode") == "rate", f"Expected mode=rate, got {resp.get('mode')}"
    print(f"  ✅ Mode set: {resp['mode']}")


def test_send_command(c: QuadSimTestClient, duration_sec: float = 3.0):
    """Step 4: Send rate commands for a few seconds."""
    print(f"\n=== Step 4: Send Commands ({duration_sec}s of yaw=30, throttle=0.44) ===")

    start = time.time()
    cmd_count = 0

    while time.time() - start < duration_sec:
        resp = c.assert_ok(
            c.send("send_command", x=0.0, y=0.0, z=30.0, w=0.44),
            "send_command"
        )
        cmd_count += 1
        time.sleep(0.02)  # ~50 Hz

    print(f"  ✅ Sent {cmd_count} commands over {duration_sec}s")


def test_send_command_with_mode(c: QuadSimTestClient, duration_sec: float = 2.0):
    """Step 4b: Send command with inline mode override (sustained)."""
    print(f"\n=== Step 4b: Send Command with Mode Override — angle ({duration_sec}s) ===")

    start = time.time()
    cmd_count = 0

    while time.time() - start < duration_sec:
        resp = c.assert_ok(
            c.send("send_command", x=5.0, y=0.0, z=0.0, w=0.44, mode="angle"),
            "send_command(mode=angle)"
        )
        cmd_count += 1
        time.sleep(0.02)

    print(f"  ✅ Sent {cmd_count} angle commands over {duration_sec}s")

    # Verify mode actually changed
    telem = c.assert_ok(c.send("get_telemetry"), "get_telemetry after mode override")
    assert telem.get("mode") == "angle", f"Expected mode=angle, got {telem.get('mode')}"
    print(f"  ✅ Mode confirmed: {telem['mode']}")

    # Switch back to rate
    c.assert_ok(c.send("set_mode", mode="rate"), "set_mode back to rate")


def test_get_sensor_data(c: QuadSimTestClient):
    """Step 5: Read sensor data and verify structure."""
    print("\n=== Step 5: Get Sensor Data ===")
    resp = c.assert_ok(c.send("get_sensor_data"), "get_sensor_data")

    # Verify expected fields exist
    required_fields = [
        "imu_ang_vel", "imu_attitude", "imu_accel", "imu_vel",
        "imu_orientation", "imu_timestamp", "imu_valid",
        "gps_position", "gps_timestamp", "gps_valid"
    ]
    for field in required_fields:
        assert field in resp, f"Missing field: {field}"

    print(f"  ✅ Sensor data received:")
    print(f"     IMU valid={resp['imu_valid']}, "
          f"attitude={fmt_vec3(resp['imu_attitude'])}")
    print(f"     GPS valid={resp['gps_valid']}, "
          f"position={fmt_vec3(resp['gps_position'])}")

    # After sending commands, angular velocity should be non-zero
    ang_vel = resp["imu_ang_vel"]
    if resp["imu_valid"]:
        mag = sum(v**2 for v in ang_vel) ** 0.5
        if mag > 0.01:
            print(f"     Angular velocity magnitude: {mag:.3f} rad/s ✅")
        else:
            print(f"     ⚠ Angular velocity near zero — drone may not be responding to commands")


def test_get_telemetry(c: QuadSimTestClient):
    """Step 6: Read telemetry and verify controller outputs."""
    print("\n=== Step 6: Get Telemetry ===")
    resp = c.assert_ok(c.send("get_telemetry"), "get_telemetry")

    required_fields = [
        "drone_id", "mode", "controller", "motors",
        "desired_rates_deg", "desired_angles_deg",
        "desired_vel", "external_cmd"
    ]
    for field in required_fields:
        assert field in resp, f"Missing field: {field}"

    motors = resp["motors"]
    print(f"  ✅ Telemetry received:")
    print(f"     drone_id={resp['drone_id']}, mode={resp['mode']}, "
          f"controller={resp['controller']}")
    print(f"     motors=[{motors[0]:.3f}, {motors[1]:.3f}, "
          f"{motors[2]:.3f}, {motors[3]:.3f}]")


def test_set_controller(c: QuadSimTestClient):
    """Step 7: Set controller type."""
    print("\n=== Step 7: Set Controller ===")

    # Set to cascade (may already be cascade — that's fine, SetController returns false
    # when already set, but the RPC handler still returns ok with current state)
    resp = c.assert_ok(c.send("set_controller", controller="cascade"), "set_controller(cascade)")
    print(f"  ✅ Controller: {resp.get('controller')}")


def test_resets(c: QuadSimTestClient):
    """Step 8: Test all reset operations."""
    print("\n=== Step 8: Reset Operations ===")

    # reset_controller — clear PIDs
    resp = c.assert_ok(c.send("reset_controller"), "reset_controller")
    print("  ✅ reset_controller")

    # reset_physics — zero velocities
    resp = c.assert_ok(c.send("reset_physics"), "reset_physics")
    print("  ✅ reset_physics")

    # reset_rotation — level the drone
    resp = c.assert_ok(c.send("reset_rotation"), "reset_rotation")
    print("  ✅ reset_rotation")

    # reset_pose — move to specific position
    resp = c.assert_ok(
        c.send("reset_pose", x=0.0, y=3.0, z=0.0, qx=0, qy=0, qz=0, qw=1),
        "reset_pose"
    )
    print("  ✅ reset_pose(0, 3, 0)")

    # Verify position after reset
    time.sleep(0.1)  # give a frame for the reset to apply
    sensors = c.assert_ok(c.send("get_sensor_data"), "get_sensor_data after reset")
    gps = sensors.get("gps_position", [0, 0, 0])
    print(f"     Post-reset GPS: {fmt_vec3(gps)}")

    # reset_all — full reset
    resp = c.assert_ok(c.send("reset_all"), "reset_all")
    print("  ✅ reset_all")


def test_time_control(c: QuadSimTestClient):
    """Step 9: Test pause / resume / step / time_scale."""
    print("\n=== Step 9: Time Control ===")

    # Get initial time
    status = c.assert_ok(c.send("get_status"), "get_status pre-pause")
    t0 = status["sim_time"]
    print(f"  Initial sim_time: {t0:.4f}")

    # Pause
    resp = c.assert_ok(c.send("pause"), "pause")
    print("  ✅ Paused")

    # Verify paused
    status = c.assert_ok(c.send("get_status"), "get_status after pause")
    assert status["is_paused"] is True, "Expected is_paused=True"

    # Step once
    resp = c.assert_ok(c.send("step", count=1), "step(1)")
    print(f"  ✅ Stepped 1: sim_time={resp.get('sim_time', '?')}")

    # Step multiple
    resp = c.assert_ok(c.send("step", count=10), "step(10)")
    print(f"  ✅ Stepped 10: sim_time={resp.get('sim_time', '?')}")

    # Resume
    resp = c.assert_ok(c.send("resume"), "resume")
    print("  ✅ Resumed")

    # Verify not paused
    status = c.assert_ok(c.send("get_status"), "get_status after resume")
    assert status["is_paused"] is False, "Expected is_paused=False"

    # Set time scale
    resp = c.assert_ok(c.send("set_time_scale", scale=2.0), "set_time_scale(2.0)")
    print(f"  ✅ Time scale set: {resp.get('time_scale')}")

    # Reset time scale back
    resp = c.assert_ok(c.send("set_time_scale", scale=1.0), "set_time_scale(1.0)")
    print(f"  ✅ Time scale reset: {resp.get('time_scale')}")


def test_reset_simulation(c: QuadSimTestClient):
    """Step 10: Reset the entire simulation."""
    print("\n=== Step 10: Reset Simulation ===")
    resp = c.assert_ok(c.send("reset_simulation"), "reset_simulation")
    print("  ✅ Simulation reset")

    # Check that sim time is near zero
    time.sleep(0.1)
    status = c.assert_ok(c.send("get_status"), "get_status after sim reset")
    print(f"     Post-reset sim_time: {status['sim_time']:.4f}")


def test_error_cases(c: QuadSimTestClient):
    """Step 11: Verify error responses for invalid inputs."""
    print("\n=== Step 11: Error Cases ===")

    # Invalid mode string
    resp = c.send("set_mode", mode="banana")
    assert resp["status"] == "error", "Expected error for invalid mode"
    print(f"  ✅ Invalid mode rejected: {resp.get('error_msg')}")

    # Invalid controller string
    resp = c.send("set_controller", controller="quantum")
    assert resp["status"] == "error", "Expected error for invalid controller"
    print(f"  ✅ Invalid controller rejected: {resp.get('error_msg')}")

    # Missing mode parameter
    resp = c.send("set_mode")
    assert resp["status"] == "error", "Expected error for missing mode"
    print(f"  ✅ Missing mode rejected: {resp.get('error_msg')}")

    # Invalid step count
    resp = c.send("step", count=0)
    assert resp["status"] == "error", "Expected error for count=0"
    print(f"  ✅ Invalid step count rejected: {resp.get('error_msg')}")

    # Invalid time scale
    resp = c.send("set_time_scale", scale=-1.0)
    assert resp["status"] == "error", "Expected error for negative scale"
    print(f"  ✅ Negative time scale rejected: {resp.get('error_msg')}")

    # Unknown method
    resp = c.send("fly_to_moon")
    assert resp["status"] == "error", "Expected error for unknown method"
    print(f"  ✅ Unknown method rejected: {resp.get('error_msg')}")


def test_disconnect(c: QuadSimTestClient):
    """Step 12: Clean disconnect."""
    print("\n=== Step 12: Disconnect ===")
    resp = c.assert_ok(c.send("disconnect"), "disconnect")
    print("  ✅ Disconnected")


# ============================================================================
# Formatting Helpers
# ============================================================================

def fmt_vec3(v) -> str:
    """Format a 3-element list/tuple as a pretty string."""
    if v and len(v) >= 3:
        return f"({v[0]:.3f}, {v[1]:.3f}, {v[2]:.3f})"
    return str(v)


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="QuadSim Phase 9b Test Script")
    parser.add_argument("--host", default="localhost", help="QuadSim host (default: localhost)")
    parser.add_argument("--port", type=int, default=5555, help="Command port (default: 5555)")
    args = parser.parse_args()

    print("=" * 60)
    print("  QuadSim Phase 9b — Command & Query Test Suite")
    print("=" * 60)

    c = QuadSimTestClient(host=args.host, port=args.port)

    try:
        c.connect_socket()

        test_connection(c)
        test_get_status(c)
        test_set_mode(c)
        test_send_command(c, duration_sec=3.0)
        test_send_command_with_mode(c)
        test_get_sensor_data(c)
        test_get_telemetry(c)
        test_set_controller(c)
        test_resets(c)
        test_time_control(c)
        test_reset_simulation(c)
        test_error_cases(c)
        test_disconnect(c)

        print("\n" + "=" * 60)
        print("  ✅ All Phase 9b tests passed!")
        print("=" * 60)

    except zmq.error.Again:
        print("\n  ❌ TIMEOUT — No response from QuadSim. Is the simulation running?")
        sys.exit(1)
    except Exception as e:
        print(f"\n  ❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        c.close()


if __name__ == "__main__":
    main()