# quadsim/future.py
# Phase 10: Lightweight Async Command Handle
#
# PURPOSE:
#   Returned by async variants of high-level commands (takeoff_async,
#   fly_to_async, etc.). Allows the caller to poll for completion,
#   block until done, or cancel an in-progress command.
#
# WHY NOT asyncio:
#   QuadSim's users are robotics researchers who write polling loops
#   and blocking scripts, not async/await web apps. A simple Future
#   built on threading.Event is the right abstraction.
#
# THREADING:
#   Each Future owns a daemon thread that runs the control loop.
#   The thread checks _cancelled each tick and exits cleanly.
#   When a new command interrupts an active Future, the old one is
#   cancelled and joined before the new thread starts.

from __future__ import annotations

import threading
from typing import Callable, Optional

from .exceptions import QuadSimError, TimeoutError


class Future:
    """
    Handle for an in-progress async flight command.

    Returned by methods like ``takeoff_async()``, ``fly_to_async()``, etc.
    Allows polling, blocking, and cancellation.

    Usage::

        future = api.fly_to_async(10, 0, 3)

        # Poll
        while not future.done:
            print(f"Position: {api.get_position()}")
            time.sleep(0.1)

        # Or block
        future.wait(timeout=30.0)

        # Or cancel
        future.cancel()

    Attributes:
        done: True when the command has completed (success or failure).
        cancelled: True if cancel() was called.
        exception: The exception if the command failed, else None.
    """

    def __init__(self) -> None:
        self._done_event = threading.Event()
        self._cancelled_event = threading.Event()
        self._exception: Optional[Exception] = None
        self._thread: Optional[threading.Thread] = None

    # ====================================================================
    # Public Interface
    # ====================================================================

    @property
    def done(self) -> bool:
        """True when the command has finished (success, failure, or cancel)."""
        return self._done_event.is_set()

    @property
    def cancelled(self) -> bool:
        """True if cancel() was called."""
        return self._cancelled_event.is_set()

    @property
    def exception(self) -> Optional[Exception]:
        """The exception that caused failure, or None if successful."""
        return self._exception

    def wait(self, timeout: Optional[float] = None) -> None:
        """
        Block until the command finishes.

        Args:
            timeout: Maximum seconds to wait. None = wait forever.

        Raises:
            TimeoutError: If timeout expires before the command finishes.
            QuadSimError: Re-raises whatever exception killed the command.
        """
        signaled = self._done_event.wait(timeout=timeout)
        if not signaled:
            raise TimeoutError(
                f"Future.wait() timed out after {timeout}s"
            )
        if self._exception is not None:
            raise self._exception

    def cancel(self) -> None:
        """
        Signal the command to stop.

        The control loop will exit on its next tick and the drone will
        switch to position hold at its current location. Does not block —
        call wait() after cancel() if you need to ensure the thread has
        exited.
        """
        self._cancelled_event.set()

    # ====================================================================
    # Internal — Used by QuadSimApi
    # ====================================================================

    def _is_cancel_requested(self) -> bool:
        """Check if cancellation has been requested. Called by control loops."""
        return self._cancelled_event.is_set()

    def _complete(self, exception: Optional[Exception] = None) -> None:
        """
        Mark the future as done.

        Args:
            exception: If not None, the command failed with this error.
        """
        self._exception = exception
        self._done_event.set()

    def _start_thread(self, target: Callable[[], None], name: str) -> None:
        """
        Start the background thread that runs the control loop.

        Args:
            target: The function to run (should call _complete when done).
            name: Thread name for debugging.
        """
        self._thread = threading.Thread(
            target=target,
            name=name,
            daemon=True,
        )
        self._thread.start()

    def _join(self, timeout: float = 5.0) -> None:
        """
        Wait for the background thread to finish. Used internally when
        cancelling to ensure clean handoff before starting a new command.
        """
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=timeout)