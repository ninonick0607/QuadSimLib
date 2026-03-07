# quadsim/future.py
# Lightweight async command handle.

from __future__ import annotations

import threading
from typing import Callable, Optional

from .exceptions import QuadSimError, TimeoutError


class Future:
    """
    Handle for an in-progress async flight command.

    Usage::

        future = drone.fly_to_async(10, 0, 3)

        while not future.done:
            print(drone.get_position())
            time.sleep(0.1)

        future.wait(timeout=30.0)
        future.cancel()
    """

    def __init__(self) -> None:
        self._done_event = threading.Event()
        self._cancelled_event = threading.Event()
        self._exception: Optional[Exception] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def done(self) -> bool:
        return self._done_event.is_set()

    @property
    def cancelled(self) -> bool:
        return self._cancelled_event.is_set()

    @property
    def exception(self) -> Optional[Exception]:
        return self._exception

    def wait(self, timeout: Optional[float] = None) -> None:
        signaled = self._done_event.wait(timeout=timeout)
        if not signaled:
            raise TimeoutError(f"Future.wait() timed out after {timeout}s")
        if self._exception is not None:
            raise self._exception

    def cancel(self) -> None:
        self._cancelled_event.set()

    # ── Internal ──

    def _is_cancel_requested(self) -> bool:
        return self._cancelled_event.is_set()

    def _complete(self, exception: Optional[Exception] = None) -> None:
        self._exception = exception
        self._done_event.set()

    def _start_thread(self, target: Callable[[], None], name: str) -> None:
        self._thread = threading.Thread(target=target, name=name, daemon=True)
        self._thread.start()

    def _join(self, timeout: float = 5.0) -> None:
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=timeout)