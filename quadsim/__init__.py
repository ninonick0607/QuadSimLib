# quadsim/__init__.py
# QuadSim Python SDK
#
# Two public classes:
#   QuadSim  — sim/world entry point, connection owner
#   Drone    — per-drone control (low-level + high-level on one object)
#
# Usage:
#   from quadsim import QuadSim
#
#   with QuadSim() as sim:
#       drone = sim.drone()
#       drone.takeoff(3.0)
#       drone.fly_to(5, 0, 3)
#       drone.send_command(roll=0, pitch=0, yaw=30, throttle=0.44)
#       sim.pause()
#       drone.land()

"""QuadSim Python SDK — research-oriented drone simulation control."""

__version__ = "0.11.0"

from .sim import QuadSim
from .drone import Drone
from .future import Future
from .types import SensorData, SimStatus, Telemetry
from .exceptions import (
    QuadSimError,
    ConnectionError,
    CommandError,
    TimeoutError,
    ProtocolError,
)

__all__ = [
    # Public API
    "QuadSim",
    "Drone",
    "Future",
    # Data types
    "SensorData",
    "Telemetry",
    "SimStatus",
    # Exceptions
    "QuadSimError",
    "ConnectionError",
    "CommandError",
    "TimeoutError",
    "ProtocolError",
]