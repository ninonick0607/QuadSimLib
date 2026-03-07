"""
QuadSim SDK — Phase 10 Unit Tests

Tests the high-level QuadSimApi using a mock QuadSimClient.
No live Unity server required.

Run:
    pytest tests/test_high_level.py -v
"""

import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
from unittest.mock import MagicMock, patch

import pytest

from quadsim.api import QuadSimApi
from quadsim.exceptions import CommandError, QuadSimError, TimeoutError
from quadsim.future import Future
from quadsim.types import SensorData, SimStatus, Telemetry


# ============================================================================
# Mock Client
# ============================================================================


class MockClient:
    """
    A mock QuadSimClient that simulates a drone at a configurable position.

    The mock tracks:
    - Current position (moves toward targets when send_command is called)
    - Current mode
    - Command history
    - Whether connect/disconnect were called
    """

    def __init__(self):
        self.connected = False
        self._position = [0.0, 0.0, 0.0]  # x, y, z (Y-up)
        self._attitude = [0.0, 0.0, 0.0]  # roll, pitch, yaw (deg)
        self._velocity = [0.0, 0.0, 0.0]  # vx, vy, vz (m/s)
        self._mode = "none"
        self._controller = "cascade"
        self.command_history: List[Dict[str, Any]] = []
        self.mode_history: List[str] = []

    def connect(self) -> SimStatus:
        self.connected = True
        return SimStatus.from_dict({
            "is_paused": False,
            "time_scale": 1.0,
            "sim_time": 0.0,
            "fixed_dt": 0.02,
            "authority": "external",
            "ui_status": "disconnected",
            "internal_status": "disconnected",
            "external_status": "connected",
            "client_connected": True,
            "client_name": "MockClient",
        })

    def disconnect(self) -> None:
        self.connected = False

    def get_sensors(self) -> SensorData:
        return SensorData(
            imu_ang_vel=(0.0, 0.0, 0.0),
            imu_attitude=tuple(self._attitude),
            imu_accel=(0.0, 0.0, 0.0),
            imu_vel=tuple(self._velocity),
            imu_orientation=(0.0, 0.0, 0.0, 1.0),
            imu_timestamp=time.monotonic(),
            imu_valid=True,
            gps_position=tuple(self._position),
            gps_timestamp=time.monotonic(),
            gps_valid=True,
        )

    def get_telemetry(self) -> Telemetry:
        return Telemetry(
            drone_id="mock-drone",
            mode=self._mode,
            controller=self._controller,
            motors=(0.0, 0.0, 0.0, 0.0),
            desired_rates_deg=(0.0, 0.0, 0.0),
            desired_angles_deg=(0.0, 0.0, 0.0),
            desired_vel=(0.0, 0.0, 0.0),
            external_cmd=(0.0, 0.0, 0.0, 0.0),
        )

    def get_status(self) -> SimStatus:
        return SimStatus.from_dict({
            "is_paused": False,
            "time_scale": 1.0,
            "sim_time": 0.0,
            "fixed_dt": 0.02,
            "authority": "external",
        })

    def set_mode(self, mode: str) -> str:
        self._mode = mode
        self.mode_history.append(mode)
        return mode

    def set_controller(self, controller: str) -> str:
        self._controller = controller
        return controller

    def send_command(self, x=0.0, y=0.0, z=0.0, w=0.0, **kwargs) -> None:
        mode = kwargs.get("mode", self._mode)
        if "mode" in kwargs and kwargs["mode"]:
            self._mode = kwargs["mode"]

        self.command_history.append({
            "x": x, "y": y, "z": z, "w": w,
            "mode": mode,
            **{k: v for k, v in kwargs.items() if k != "mode"},
        })

        # Simulate drone movement based on commands
        # This is a very rough simulation — just enough for tests
        if mode == "velocity":
            # Move in the direction of the velocity command
            dt = 0.02  # one tick
            vx = kwargs.get("vx", x)
            vy = kwargs.get("vy", y)
            vz = kwargs.get("vz", z)
            self._position[0] += vx * dt
            self._position[1] += vy * dt
            self._position[2] += vz * dt
        elif mode == "position":
            # Move toward target at a realistic rate — max 2m per tick
            # This prevents distant targets from being reached instantly
            max_step = 0.1  # meters per tick
            for i, target in enumerate([x, y, z]):
                diff = target - self._position[i]
                if abs(diff) > max_step:
                    self._position[i] += max_step * (1.0 if diff > 0 else -1.0)
                else:
                    self._position[i] = target
            # Yaw
            yaw_diff = w - self._attitude[2]
            if abs(yaw_diff) > 1.0:
                self._attitude[2] += 1.0 * (1.0 if yaw_diff > 0 else -1.0)
            else:
                self._attitude[2] = w

    def pause(self) -> None:
        pass

    def resume(self) -> None:
        pass

    def step(self, count=1) -> dict:
        return {"steps": count}

    def set_time_scale(self, scale: float) -> float:
        return scale

    def reset(self) -> None:
        self._position = [0.0, 0.0, 0.0]
        self._attitude = [0.0, 0.0, 0.0]

    def reset_simulation(self) -> None:
        self.reset()

    def subscribe(self, callback, topics=None, hz=50.0) -> None:
        pass

    def unsubscribe(self) -> None:
        pass


def make_api_with_mock(start_pos=(0.0, 0.0, 0.0)) -> Tuple[QuadSimApi, MockClient]:
    """Create a QuadSimApi with a MockClient, bypassing real connection."""
    api = QuadSimApi.__new__(QuadSimApi)
    mock = MockClient()
    mock._position = list(start_pos)
    mock.connected = True

    # Initialize the fields that __init__ would set
    api.client = mock
    api.hover_throttle = 0.44
    api.position_tolerance = 0.5
    api.altitude_tolerance = 0.3
    api.landing_altitude = 0.15
    api.default_speed = 2.0
    api.control_loop_hz = 200.0  # Fast for tests
    api.default_leg_timeout = 30.0
    api.use_velocity_mode_navigation = False
    api._active_future = None
    api._command_lock = threading.Lock()

    return api, mock


# ============================================================================
# Tests: Construction & Convenience Queries
# ============================================================================


class TestConstruction:
    def test_default_params(self):
        api = QuadSimApi()
        assert api.position_tolerance == 0.5
        assert api.altitude_tolerance == 0.3
        assert api.landing_altitude == 0.15
        assert api.control_loop_hz == 50.0
        assert api.use_velocity_mode_navigation is False

    def test_custom_params(self):
        api = QuadSimApi(host="192.168.1.10", command_port=6666)
        assert api.client._host == "192.168.1.10"
        assert api.client._command_port == 6666


class TestConvenienceQueries:
    def test_get_position(self):
        api, mock = make_api_with_mock(start_pos=(1.0, 2.0, 3.0))
        pos = api.get_position()
        assert pos == (1.0, 2.0, 3.0)

    def test_get_attitude(self):
        api, mock = make_api_with_mock()
        mock._attitude = [10.0, 5.0, 45.0]
        att = api.get_attitude()
        assert att == (10.0, 5.0, 45.0)

    def test_get_velocity(self):
        api, mock = make_api_with_mock()
        mock._velocity = [1.0, 0.5, -0.3]
        vel = api.get_velocity()
        assert vel == (1.0, 0.5, -0.3)

    def test_is_airborne_true(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        assert api.is_airborne() is True

    def test_is_airborne_false(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 0.05, 0.0))
        assert api.is_airborne() is False


# ============================================================================
# Tests: Takeoff
# ============================================================================


class TestTakeoff:
    def test_takeoff_reaches_altitude(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 0.0, 0.0))
        api.takeoff(altitude=3.0, speed=1.0, timeout=5.0)

        # After takeoff, drone should be near target altitude
        pos = api.get_position()
        assert abs(pos[1] - 3.0) < api.altitude_tolerance

    def test_takeoff_sets_velocity_mode(self):
        api, mock = make_api_with_mock()
        api.takeoff(altitude=3.0, timeout=5.0)

        # Should have set velocity mode during climb
        assert "velocity" in mock.mode_history

    def test_takeoff_switches_to_position_hold(self):
        api, mock = make_api_with_mock()
        api.takeoff(altitude=3.0, timeout=5.0)

        # Last mode should be position (for hold)
        last_cmd = mock.command_history[-1]
        assert last_cmd["mode"] == "position"

    def test_takeoff_timeout(self):
        api, mock = make_api_with_mock()
        # Make the mock not move at all
        original_send = mock.send_command
        mock.send_command = lambda **kwargs: None

        with pytest.raises(TimeoutError, match="takeoff"):
            api.takeoff(altitude=100.0, timeout=0.5)


# ============================================================================
# Tests: Land
# ============================================================================


class TestLand:
    def test_land_from_altitude(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        api.land(speed=1.0, timeout=5.0)

        pos = api.get_position()
        assert pos[1] <= api.landing_altitude

    def test_land_uses_velocity_mode(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        api.land(speed=1.0, timeout=5.0)
        assert "velocity" in mock.mode_history

    def test_land_already_grounded(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 0.05, 0.0))
        api.land(timeout=2.0)
        # Should complete almost immediately
        pos = api.get_position()
        assert pos[1] <= api.landing_altitude


# ============================================================================
# Tests: Hover
# ============================================================================


class TestHover:
    def test_hover_holds_position(self):
        api, mock = make_api_with_mock(start_pos=(5.0, 3.0, 2.0))
        start_pos = api.get_position()

        api.hover(duration=0.2)  # Short duration for test speed

        end_pos = api.get_position()
        # Position should be roughly the same
        assert abs(end_pos[0] - start_pos[0]) < 1.0
        assert abs(end_pos[1] - start_pos[1]) < 1.0

    def test_hover_uses_position_mode(self):
        api, mock = make_api_with_mock(start_pos=(5.0, 3.0, 2.0))
        api.hover(duration=0.1)
        assert "position" in mock.mode_history


# ============================================================================
# Tests: Fly To
# ============================================================================


class TestFlyTo:
    def test_fly_to_position_mode(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        api.fly_to(x=5.0, y=3.0, z=5.0, timeout=5.0)

        pos = api.get_position()
        assert abs(pos[0] - 5.0) < api.position_tolerance
        assert abs(pos[2] - 5.0) < api.position_tolerance

    def test_fly_to_velocity_fallback(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        api.use_velocity_mode_navigation = True
        api.fly_to(x=5.0, y=3.0, z=5.0, speed=2.0, timeout=5.0)

        assert "velocity" in mock.mode_history

    def test_fly_to_uses_default_speed(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        api.default_speed = 5.0
        api.fly_to(x=1.0, y=3.0, z=0.0, timeout=5.0)
        # Just verify it doesn't crash — speed is internal to the controller


# ============================================================================
# Tests: Fly Path
# ============================================================================


class TestFlyPath:
    def test_fly_path_visits_all_waypoints(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        waypoints = [
            (5.0, 3.0, 0.0),
            (5.0, 3.0, 5.0),
            (0.0, 3.0, 5.0),
        ]
        api.fly_path(waypoints, speed=2.0, timeout=30.0)

        # Should be near the last waypoint
        pos = api.get_position()
        assert abs(pos[0] - 0.0) < api.position_tolerance
        assert abs(pos[2] - 5.0) < api.position_tolerance

    def test_fly_path_empty_raises(self):
        api, mock = make_api_with_mock()
        with pytest.raises(ValueError, match="empty"):
            api.fly_path([], speed=2.0)


# ============================================================================
# Tests: Yaw To
# ============================================================================


class TestYawTo:
    def test_yaw_to_target(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        api.yaw_to(heading_deg=90.0, timeout=5.0)

        att = api.get_attitude()
        assert abs(att[2] - 90.0) < 5.0  # Within 5 degrees


# ============================================================================
# Tests: Future
# ============================================================================


class TestFuture:
    def test_future_basics(self):
        f = Future()
        assert f.done is False
        assert f.cancelled is False
        assert f.exception is None

    def test_future_complete_success(self):
        f = Future()
        f._complete()
        assert f.done is True
        assert f.exception is None

    def test_future_complete_with_error(self):
        f = Future()
        err = QuadSimError("test error")
        f._complete(exception=err)
        assert f.done is True
        assert f.exception is err

    def test_future_wait_success(self):
        f = Future()
        f._complete()
        f.wait(timeout=1.0)  # Should not raise

    def test_future_wait_reraises(self):
        f = Future()
        f._complete(exception=CommandError("rejected"))
        with pytest.raises(CommandError, match="rejected"):
            f.wait()

    def test_future_wait_timeout(self):
        f = Future()
        with pytest.raises(TimeoutError, match="timed out"):
            f.wait(timeout=0.1)

    def test_future_cancel(self):
        f = Future()
        f.cancel()
        assert f.cancelled is True
        assert f._is_cancel_requested() is True


# ============================================================================
# Tests: Async Commands
# ============================================================================


class TestAsync:
    def test_async_takeoff(self):
        api, mock = make_api_with_mock()
        future = api.takeoff_async(altitude=3.0, timeout=5.0)

        future.wait(timeout=10.0)
        assert future.done is True
        assert future.exception is None

        pos = api.get_position()
        assert abs(pos[1] - 3.0) < api.altitude_tolerance

    def test_async_fly_to(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        future = api.fly_to_async(x=5.0, y=3.0, z=0.0, timeout=5.0)
        future.wait(timeout=10.0)
        assert future.done

    def test_async_cancel(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        future = api.fly_to_async(x=100.0, y=3.0, z=100.0, speed=0.1, timeout=60.0)

        time.sleep(0.1)
        future.cancel()
        future.wait(timeout=5.0)  # Should return quickly after cancel

        assert future.done is True
        assert future.cancelled is True

    def test_async_interruption(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))

        # Start a long flight
        future1 = api.fly_to_async(x=100.0, y=3.0, z=100.0, speed=0.1, timeout=60.0)
        time.sleep(0.1)

        # Interrupt with a new command — old future should be cancelled
        future2 = api.fly_to_async(x=1.0, y=3.0, z=0.0, timeout=5.0)

        assert future1.cancelled is True
        future2.wait(timeout=10.0)
        assert future2.done is True


# ============================================================================
# Tests: Sim Control Passthroughs
# ============================================================================


class TestSimControl:
    def test_pause_resume(self):
        api, mock = make_api_with_mock()
        api.pause()   # Should not raise
        api.resume()  # Should not raise

    def test_step(self):
        api, mock = make_api_with_mock()
        result = api.step(count=5)
        assert result == {"steps": 5}

    def test_set_time_scale(self):
        api, mock = make_api_with_mock()
        scale = api.set_time_scale(2.0)
        assert scale == 2.0

    def test_reset_cancels_active(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        future = api.fly_to_async(x=100.0, y=3.0, z=100.0, speed=0.1, timeout=60.0)
        time.sleep(0.1)

        api.reset()

        assert future.cancelled is True


# ============================================================================
# Tests: Edge Cases
# ============================================================================


class TestEdgeCases:
    def test_disconnect_cancels_active(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        future = api.fly_to_async(x=100.0, y=3.0, z=100.0, speed=0.1, timeout=60.0)
        time.sleep(0.1)

        api.disconnect()
        assert future.cancelled is True

    def test_safe_position_hold_on_timeout(self):
        api, mock = make_api_with_mock(start_pos=(5.0, 3.0, 2.0))
        # Make send_command a no-op so fly_to never arrives
        mock.send_command = lambda **kwargs: None

        with pytest.raises(TimeoutError):
            api.fly_to(x=100.0, y=3.0, z=100.0, timeout=0.3)

    def test_configurable_tolerance(self):
        api, mock = make_api_with_mock(start_pos=(0.0, 3.0, 0.0))
        api.position_tolerance = 2.0  # Very generous
        api.fly_to(x=1.5, y=3.0, z=0.0, timeout=5.0)
        # Should arrive quickly with generous tolerance