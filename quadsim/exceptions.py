"""
quadsim.exceptions — Error types for the QuadSim Python client.

Hierarchy:
    QuadSimError
    ├── ConnectionError    — connect/disconnect/heartbeat failures
    ├── CommandError       — command rejected (authority, invalid params)
    ├── TimeoutError       — server didn't respond in time
    └── ProtocolError      — unexpected wire format or deserialization failure
"""


class QuadSimError(Exception):
    """Base exception for all QuadSim client errors."""
    pass


class ConnectionError(QuadSimError):
    """Raised when connection lifecycle fails (connect, disconnect, heartbeat timeout)."""
    pass


class CommandError(QuadSimError):
    """Raised when a command is rejected by the server (authority, invalid params, no drone)."""
    pass


class TimeoutError(QuadSimError):
    """Raised when the server doesn't respond within the configured timeout."""
    pass


class ProtocolError(QuadSimError):
    """Raised when the wire protocol produces unexpected data."""
    pass