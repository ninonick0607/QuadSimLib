# quadsim/exceptions.py

class QuadSimError(Exception):
    """Base exception for all QuadSim errors."""
    pass

class ConnectionError(QuadSimError):
    """Connection-related errors (not connected, denied)."""
    pass

class CommandError(QuadSimError):
    """Command-related errors (authority rejected, no drone, invalid mode)."""
    pass

class TimeoutError(QuadSimError):
    """Timeout errors (RPC timeout, flight command timeout)."""
    pass

class ProtocolError(QuadSimError):
    """Wire-level protocol errors (serialization, unexpected responses)."""
    pass