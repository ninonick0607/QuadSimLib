# quadsim/exceptions.py
# Phase 9d: Exception Hierarchy
#
# PURPOSE:
#   Strongly-typed error classes for the QuadSim Python client.
#   Every exception inherits from QuadSimError so callers can catch
#   broadly or narrowly.
#
# HIERARCHY:
#   QuadSimError (base)
#   ├── ConnectionError  — connect/disconnect/heartbeat failures
#   ├── CommandError     — authority rejected, invalid params, no drone
#   ├── TimeoutError     — server didn't respond within timeout_ms
#   └── ProtocolError    — unexpected wire format
#
# NOTE:
#   ConnectionError and TimeoutError shadow Python builtins intentionally.
#   Users who need both can alias: `from quadsim.exceptions import ConnectionError as QSConnError`


class QuadSimError(Exception):
    """Base exception for all QuadSim errors."""
    pass


class ConnectionError(QuadSimError):
    """
    Connection lifecycle failures.

    Raised when:
    - connect() is denied or fails
    - disconnect() fails
    - heartbeat timeout detected
    - server reports "not_connected" or "denied"
    """
    pass


class CommandError(QuadSimError):
    """
    Command execution failures.

    Raised when:
    - Authority is rejected ("authority_rejected")
    - Invalid parameters sent
    - No drone available ("no_drone")
    - Command failed on the sim side ("command_failed")
    """
    pass


class TimeoutError(QuadSimError):
    """
    Server did not respond within the configured timeout.

    Raised when:
    - REQ/REP round-trip exceeds timeout_ms
    - A blocking high-level command (takeoff, fly_to, etc.) exceeds its timeout
    """
    pass


class ProtocolError(QuadSimError):
    """
    Unexpected wire format or deserialization failure.

    Raised when:
    - MessagePack decode fails
    - Response missing required fields
    - Unknown status value in response
    """
    pass