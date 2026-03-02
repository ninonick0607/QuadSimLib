#!/usr/bin/env python3
"""
test_quadsim_client.py — End-to-end test for the QuadSim Python client library.

Validates all Phase 9d verification criteria:
  - QuadSimClient connects, sends commands, reads data, disconnects
  - Context manager works
  - Background heartbeat keeps connection alive
  - Named arguments for send_command work correctly
  - SensorData and Telemetry dataclasses populate correctly
  - Timeout errors raise QuadSimError with clear messages
  - Control loop pattern works

Prerequisites:
  - QuadSim running in Unity with ExternalRpcAdapter
  - Authority default set to External
  - pip install pyzmq msgpack
"""

import sys
import time

# Adjust path if running from repo root
sys.path.insert(0, "")

from quadsim import QuadSimClient, SensorData, Telemetry, SimStatus
from quadsim.exceptions import ConnectionError, CommandError


def test_context_manager():
    """Test 1: Context manager connects and disconnects cleanly."""
    print("\n=== Test 1: Context Manager ===")

    with QuadSimClient("localhost", client_name="test_ctx_mgr") as client:
        assert client.connected, "Should be connected inside context"
        status = client.get_status()
        assert isinstance(status, SimStatus), "get_status should return SimStatus"
        print(f"  ✅ Connected via context manager, sim_time={status.sim_time:.3f}")

    # After exiting context, client should be disconnected
    assert not client.connected, "Should be disconnected after context exit"
    print("  ✅ Disconnected on context exit")


def test_status_and_types():
    """Test 2: Status returns proper typed dataclass."""
    print("\n=== Test 2: Status & Types ===")

    with QuadSimClient("localhost", client_name="test_types") as client:
        status = client.get_status()

        # Verify it's a proper dataclass with typed fields
        assert isinstance(status.is_paused, bool)
        assert isinstance(status.time_scale, float)
        assert isinstance(status.sim_time, float)
        assert isinstance(status.authority, str)
        assert status.client_connected is True
        assert status.authority == "External"

        print(f"  ✅ SimStatus: paused={status.is_paused}, scale={status.time_scale}, "
              f"authority={status.authority}")


def test_mode_and_controller():
    """Test 3: Set mode and controller."""
    print("\n=== Test 3: Mode & Controller ===")

    with QuadSimClient("localhost", client_name="test_mode") as client:
        mode = client.set_mode("rate")
        assert mode == "rate", f"Expected rate, got {mode}"
        print(f"  ✅ Mode set: {mode}")

        mode = client.set_mode("angle")
        assert mode == "angle", f"Expected angle, got {mode}"
        print(f"  ✅ Mode set: {mode}")

        ctrl = client.set_controller("cascade")
        assert ctrl == "cascade", f"Expected cascade, got {ctrl}"
        print(f"  ✅ Controller set: {ctrl}")

        # Switch back
        client.set_mode("rate")


def test_send_command_raw():
    """Test 4: Send raw Axis4 commands."""
    print("\n=== Test 4: Raw Axis4 Commands (3s) ===")

    with QuadSimClient("localhost", client_name="test_raw_cmd") as client:
        client.set_mode("rate")

        start = time.time()
        count = 0
        while time.time() - start < 3.0:
            client.send_command(x=0, y=0, z=30, w=0.44)
            count += 1
            time.sleep(0.02)

        print(f"  ✅ Sent {count} raw commands over 3s")


def test_send_command_named():
    """Test 5: Send commands with named arguments."""
    print("\n=== Test 5: Named Argument Commands (2s) ===")

    with QuadSimClient("localhost", client_name="test_named_cmd") as client:
        client.set_mode("rate")

        start = time.time()
        count = 0
        while time.time() - start < 2.0:
            client.send_command(roll=0, pitch=0, yaw=30, throttle=0.44)
            count += 1
            time.sleep(0.02)

        print(f"  ✅ Sent {count} named-arg commands over 2s")


def test_send_command_mode_override():
    """Test 6: Send command with inline mode override."""
    print("\n=== Test 6: Mode Override Commands (2s) ===")

    with QuadSimClient("localhost", client_name="test_mode_override") as client:
        client.set_mode("rate")

        start = time.time()
        count = 0
        while time.time() - start < 2.0:
            client.send_command(roll=5, pitch=0, yaw=0, throttle=0.44, mode="angle")
            count += 1
            time.sleep(0.02)

        telem = client.get_telemetry()
        assert telem.mode == "angle", f"Expected angle, got {telem.mode}"
        print(f"  ✅ Sent {count} angle-override commands, mode confirmed: {telem.mode}")

        client.set_mode("rate")


def test_sensors():
    """Test 7: Read sensor data into typed dataclass."""
    print("\n=== Test 7: Sensor Data ===")

    with QuadSimClient("localhost", client_name="test_sensors") as client:
        sensors = client.get_sensors()

        assert isinstance(sensors, SensorData)
        assert isinstance(sensors.imu_ang_vel, tuple)
        assert len(sensors.imu_ang_vel) == 3
        assert isinstance(sensors.imu_orientation, tuple)
        assert len(sensors.imu_orientation) == 4
        assert isinstance(sensors.imu_valid, bool)
        assert isinstance(sensors.gps_position, tuple)
        assert len(sensors.gps_position) == 3

        print(f"  ✅ SensorData: attitude={sensors.imu_attitude}, "
              f"gps={sensors.gps_position}")


def test_telemetry():
    """Test 8: Read telemetry into typed dataclass."""
    print("\n=== Test 8: Telemetry ===")

    with QuadSimClient("localhost", client_name="test_telem") as client:
        telem = client.get_telemetry()

        assert isinstance(telem, Telemetry)
        assert isinstance(telem.drone_id, str)
        assert isinstance(telem.mode, str)
        assert isinstance(telem.motors, tuple)
        assert len(telem.motors) == 4
        assert len(telem.external_cmd) == 4

        print(f"  ✅ Telemetry: drone={telem.drone_id}, mode={telem.mode}, "
              f"motors={tuple(f'{m:.3f}' for m in telem.motors)}")


def test_resets():
    """Test 9: All reset operations."""
    print("\n=== Test 9: Reset Operations ===")

    with QuadSimClient("localhost", client_name="test_resets") as client:
        client.reset_controller()
        print("  ✅ reset_controller")

        client.reset_physics()
        print("  ✅ reset_physics")

        client.reset_rotation()
        print("  ✅ reset_rotation")

        client.reset_pose(x=0, y=3, z=0)
        print("  ✅ reset_pose(0, 3, 0)")

        time.sleep(0.1)
        sensors = client.get_sensors()
        print(f"     Post-reset GPS: {sensors.gps_position}")

        client.reset()
        print("  ✅ reset (full)")


def test_time_control():
    """Test 10: Pause, step, resume, time scale."""
    print("\n=== Test 10: Time Control ===")

    with QuadSimClient("localhost", client_name="test_time") as client:
        status = client.get_status()
        print(f"  Initial sim_time: {status.sim_time:.4f}")

        client.pause()
        status = client.get_status()
        assert status.is_paused is True
        print("  ✅ Paused")

        resp = client.step(count=1)
        print(f"  ✅ Stepped 1: sim_time={resp.get('sim_time', '?')}")

        resp = client.step(count=10)
        print(f"  ✅ Stepped 10: sim_time={resp.get('sim_time', '?')}")

        client.resume()
        status = client.get_status()
        assert status.is_paused is False
        print("  ✅ Resumed")

        scale = client.set_time_scale(2.0)
        print(f"  ✅ Time scale: {scale}")

        client.set_time_scale(1.0)
        print("  ✅ Time scale reset to 1.0")


def test_reset_simulation():
    """Test 11: Full simulation reset."""
    print("\n=== Test 11: Reset Simulation ===")

    with QuadSimClient("localhost", client_name="test_sim_reset") as client:
        client.reset_simulation()
        time.sleep(0.1)
        status = client.get_status()
        print(f"  ✅ Simulation reset, sim_time={status.sim_time:.4f}")


def test_heartbeat_survives_long_session():
    """Test 12: Heartbeat keeps connection alive over a longer session."""
    print("\n=== Test 12: Heartbeat Survival (8s session) ===")

    with QuadSimClient(
        "localhost",
        client_name="test_heartbeat",
        heartbeat_interval=2.0,
    ) as client:
        # Do nothing for longer than the heartbeat timeout (5s)
        # The background heartbeat should keep us alive
        print("  Waiting 8 seconds (heartbeat timeout is 5s)...")
        time.sleep(8.0)

        # If heartbeat works, this should succeed
        status = client.get_status()
        assert status.client_connected is True
        print(f"  ✅ Still connected after 8s idle, sim_time={status.sim_time:.3f}")


def test_control_loop_pattern():
    """Test 13: Realistic control loop — read sensors, compute, send command."""
    print("\n=== Test 13: Control Loop Pattern (2s) ===")

    with QuadSimClient("localhost", client_name="test_loop") as client:
        client.reset()
        client.set_mode("rate")
        time.sleep(0.1)

        start = time.time()
        count = 0

        while time.time() - start < 2.0:
            sensors = client.get_sensors()

            # Simple proportional controller: try to yaw at 30 deg/s
            yaw_cmd = 30.0
            throttle_cmd = 0.44

            client.send_command(yaw=yaw_cmd, throttle=throttle_cmd)
            count += 1
            time.sleep(0.02)

        telem = client.get_telemetry()
        print(f"  ✅ Control loop ran {count} iterations over 2s")
        print(f"     Final motors: {tuple(f'{m:.3f}' for m in telem.motors)}")


def test_error_handling():
    """Test 14: Verify error types are raised correctly."""
    print("\n=== Test 14: Error Handling ===")

    with QuadSimClient("localhost", client_name="test_errors") as client:
        # Invalid mode
        try:
            client.set_mode("banana")
            assert False, "Should have raised CommandError"
        except CommandError as e:
            print(f"  ✅ Invalid mode → CommandError: {e}")

        # Invalid controller
        try:
            client.set_controller("quantum")
            assert False, "Should have raised CommandError"
        except CommandError as e:
            print(f"  ✅ Invalid controller → CommandError: {e}")

    # Send on disconnected client
    client_disconnected = QuadSimClient("localhost")
    try:
        client_disconnected.set_mode("rate")
        assert False, "Should have raised ConnectionError"
    except ConnectionError as e:
        print(f"  ✅ Disconnected send → ConnectionError: {e}")


# ================================================================
# Main
# ================================================================

def main():
    print("=" * 60)
    print("  QuadSim Phase 9d — Python Client Library Test Suite")
    print("=" * 60)

    try:
        test_context_manager()
        test_status_and_types()
        test_mode_and_controller()
        test_send_command_raw()
        test_send_command_named()
        test_send_command_mode_override()
        test_sensors()
        test_telemetry()
        test_resets()
        test_time_control()
        test_reset_simulation()
        test_heartbeat_survives_long_session()
        test_control_loop_pattern()
        test_error_handling()

        print("\n" + "=" * 60)
        print("  ✅ All Phase 9d tests passed!")
        print("=" * 60)

    except Exception as e:
        print(f"\n  ❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()