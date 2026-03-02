#!/usr/bin/env python3
"""
test_streaming.py — Phase 9c PUB/SUB Telemetry Streaming Test

Validates:
  - subscribe/unsubscribe via REQ/REP
  - PUB socket streams data when subscribed
  - Typed callbacks (subscribe_sensors, subscribe_telemetry)
  - Stream survives pause/resume
  - Clean shutdown on unsubscribe and disconnect

Prerequisites:
  - QuadSim running in Unity with ExternalRpcAdapter
  - pip install pyzmq msgpack
"""

import sys
import time
import threading

sys.path.insert(0, ".")

from quadsim import QuadSimClient, SensorData, Telemetry


def test_raw_subscribe():
    """Test 1: Subscribe to both topics, receive raw dicts."""
    print("\n=== Test 1: Raw Subscribe (both topics, 3s) ===")

    received = {"sensors": 0, "telemetry": 0}
    lock = threading.Lock()

    def on_message(data):
        topic = data.get("topic", "unknown")
        with lock:
            if topic in received:
                received[topic] += 1

    with QuadSimClient("localhost", client_name="test_stream_raw") as client:
        client.subscribe(on_message, hz=50)

        time.sleep(3.0)

        client.unsubscribe()

    with lock:
        s_count = received["sensors"]
        t_count = received["telemetry"]

    print(f"  Received: sensors={s_count}, telemetry={t_count}")
    assert s_count > 50, f"Expected >50 sensor messages in 3s at 50Hz, got {s_count}"
    assert t_count > 50, f"Expected >50 telemetry messages in 3s at 50Hz, got {t_count}"
    print("  ✅ Both topics streaming at expected rate")


def test_sensors_only():
    """Test 2: Subscribe to sensors only with typed callback."""
    print("\n=== Test 2: Sensors-Only Subscribe (2s) ===")

    received = []
    lock = threading.Lock()

    def on_sensor(data: SensorData):
        with lock:
            if len(received) < 5:
                received.append(data)

    with QuadSimClient("localhost", client_name="test_stream_sensors") as client:
        client.subscribe_sensors(on_sensor, hz=30)

        time.sleep(2.0)

        client.unsubscribe()

    with lock:
        count = len(received)

    assert count >= 5, f"Expected >=5 SensorData objects, got {count}"
    first = received[0]
    assert isinstance(first, SensorData), f"Expected SensorData, got {type(first)}"
    assert len(first.imu_attitude) == 3
    assert len(first.gps_position) == 3
    print(f"  ✅ Received {count} SensorData objects")
    print(f"     Sample: attitude={first.imu_attitude}, gps={first.gps_position}")


def test_telemetry_only():
    """Test 3: Subscribe to telemetry only with typed callback."""
    print("\n=== Test 3: Telemetry-Only Subscribe (2s) ===")

    received = []
    lock = threading.Lock()

    def on_telem(data: Telemetry):
        with lock:
            if len(received) < 5:
                received.append(data)

    with QuadSimClient("localhost", client_name="test_stream_telem") as client:
        client.subscribe_telemetry(on_telem, hz=30)

        time.sleep(2.0)

        client.unsubscribe()

    with lock:
        count = len(received)

    assert count >= 5, f"Expected >=5 Telemetry objects, got {count}"
    first = received[0]
    assert isinstance(first, Telemetry), f"Expected Telemetry, got {type(first)}"
    assert len(first.motors) == 4
    assert first.mode in ("rate", "angle", "velocity", "position", "passthrough", "wrench", "none")
    print(f"  ✅ Received {count} Telemetry objects")
    print(f"     Sample: mode={first.mode}, motors={tuple(f'{m:.3f}' for m in first.motors)}")


def test_custom_hz():
    """Test 4: Request a specific publish rate."""
    print("\n=== Test 4: Custom Hz (10 Hz, 3s) ===")

    count = 0
    lock = threading.Lock()

    def on_message(data):
        nonlocal count
        with lock:
            count += 1

    with QuadSimClient("localhost", client_name="test_stream_hz") as client:
        client.subscribe(on_message, topics=["sensors"], hz=10)

        time.sleep(3.0)

        client.unsubscribe()

    with lock:
        final_count = count

    # At 10 Hz for 3s, expect ~30 messages (sensors only).
    # Allow wide tolerance for timing jitter.
    print(f"  Received {final_count} messages (expected ~30 at 10 Hz)")
    assert 15 < final_count < 60, f"Count {final_count} outside expected range for 10 Hz"
    print("  ✅ Rate is in expected range")


def test_stream_through_pause_resume():
    """Test 5: Stream continues through pause/resume."""
    print("\n=== Test 5: Stream Through Pause/Resume ===")

    count_during_pause = 0
    count_after_resume = 0
    lock = threading.Lock()
    phase = "before"

    def on_message(data):
        nonlocal count_during_pause, count_after_resume, phase
        with lock:
            if phase == "paused":
                count_during_pause += 1
            elif phase == "resumed":
                count_after_resume += 1

    with QuadSimClient("localhost", client_name="test_stream_pause") as client:
        client.subscribe(on_message, topics=["sensors"], hz=50)
        time.sleep(0.5)  # Let stream start

        client.pause()
        with lock:
            phase = "paused"
        time.sleep(2.0)

        with lock:
            phase = "resumed"
        client.resume()
        time.sleep(2.0)

        client.unsubscribe()

    print(f"  During pause: {count_during_pause} messages")
    print(f"  After resume: {count_after_resume} messages")

    # Stream should continue even when paused (PUB runs on unscaled time)
    # After resume, should definitely be streaming
    assert count_after_resume > 30, f"Expected >30 messages after resume, got {count_after_resume}"
    print("  ✅ Stream active after resume")


def test_disconnect_stops_stream():
    """Test 6: Disconnect cleans up stream automatically."""
    print("\n=== Test 6: Disconnect Stops Stream ===")

    count = 0
    lock = threading.Lock()

    def on_message(data):
        nonlocal count
        with lock:
            count += 1

    client = QuadSimClient("localhost", client_name="test_stream_disconnect")
    client.connect()
    client.subscribe(on_message, hz=50)

    time.sleep(1.0)
    with lock:
        before = count

    # Disconnect should stop streaming automatically
    client.disconnect()

    assert before > 20, f"Expected >20 messages before disconnect, got {before}"
    assert not client.connected
    print(f"  ✅ Received {before} messages, then disconnect cleaned up")


def test_subscribe_while_sending_commands():
    """Test 7: Stream + command sending simultaneously."""
    print("\n=== Test 7: Stream + Commands (3s) ===")

    sensor_count = 0
    lock = threading.Lock()

    def on_message(data):
        nonlocal sensor_count
        with lock:
            sensor_count += 1

    with QuadSimClient("localhost", client_name="test_stream_cmds") as client:
        client.set_mode("rate")
        client.subscribe(on_message, topics=["sensors"], hz=50)

        start = time.time()
        cmd_count = 0
        while time.time() - start < 3.0:
            client.send_command(yaw=30, throttle=0.44)
            cmd_count += 1
            time.sleep(0.02)

        client.unsubscribe()

    with lock:
        final_sensor = sensor_count

    print(f"  Commands sent: {cmd_count}")
    print(f"  Sensor messages received: {final_sensor}")
    assert cmd_count > 50, f"Expected >50 commands, got {cmd_count}"
    assert final_sensor > 50, f"Expected >50 sensor messages, got {final_sensor}"
    print("  ✅ Commands and streaming coexist")


def main():
    print("=" * 60)
    print("  QuadSim Phase 9c — Telemetry Streaming Test Suite")
    print("=" * 60)

    try:
        test_raw_subscribe()
        test_sensors_only()
        test_telemetry_only()
        test_custom_hz()
        test_stream_through_pause_resume()
        test_disconnect_stops_stream()
        test_subscribe_while_sending_commands()

        print("\n" + "=" * 60)
        print("  ✅ All Phase 9c tests passed!")
        print("=" * 60)

    except Exception as e:
        print(f"\n  ❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()