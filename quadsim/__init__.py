# quadsim/__init__.py
# QuadSim Python SDK
#
# Two-layer API:
#   QuadSimApi    — High-level: takeoff(), fly_to(), hover(), land()
#   QuadSimClient — Low-level:  send_command(), get_sensors(), set_mode()
#
# Usage:
#   from quadsim import QuadSimApi
#   with QuadSimApi() as api:
#       api.takeoff(3.0)
#       api.fly_to(10, 3, 0)
#       api.land()

"""QuadSim Python SDK — research-oriented drone simulation control."""

__version__ = "0.10.0"

from .api import QuadSimApi
from .client import QuadSimClient
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
    # High-level API (Phase 10)
    "QuadSimApi",
    # Low-level client (Phase 9d)
    "QuadSimClient",
    # Async handle (Phase 10)
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