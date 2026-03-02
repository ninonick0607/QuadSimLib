"""
QuadSim Python Client Library
==============================

Control the QuadSim drone simulator from Python.

Quick start:
    from quadsim import QuadSimClient

    with QuadSimClient("localhost") as client:
        client.set_mode("rate")
        client.send_command(yaw=30, throttle=0.44)
        sensors = client.get_sensors()
        print(f"Position: {sensors.gps_position}")
"""

from .client import QuadSimClient
from quadsim.types import SensorData, SimStatus, Telemetry
from .exceptions import (
    QuadSimError,
    ConnectionError,
    CommandError,
    TimeoutError,
    ProtocolError,
)

__version__ = "0.1.0"

__all__ = [
    "QuadSimClient",
    "SensorData",
    "SimStatus",
    "Telemetry",
    "QuadSimError",
    "ConnectionError",
    "CommandError",
    "TimeoutError",
    "ProtocolError",
]