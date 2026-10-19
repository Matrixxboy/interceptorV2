"""Thread-safe 'latest value only' mailbox (T1.1).

LatestSlot implements overwrite semantics — the reader always gets the most recent
value without queue growth. Used for inter-thread data flow (§5).
Frames carry capture_ts_ns so the consumer can detect staleness.
"""

from __future__ import annotations

import threading
from typing import Generic, TypeVar

T = TypeVar("T")


class LatestSlot(Generic[T]):
    """Thread-safe slot that always holds the latest written value.

    Writers overwrite the current value (no queue, no backpressure).
    Readers get the most recent value and can detect if it changed.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: T | None = None
        self._version: int = 0

    def put(self, value: T) -> None:
        """Write a new value, overwriting any existing one.

        Args:
            value: The new value to store.
        """
        with self._lock:
            self._value = value
            self._version += 1

    def get(self) -> T | None:
        """Read the latest value, or None if nothing was ever written.

        Returns:
            The latest value, or None.
        """
        with self._lock:
            return self._value

    def get_versioned(self) -> tuple[T | None, int]:
        """Read the latest value along with its version number.

        The version increments on every put(). Consumers can use this
        to detect whether a new value was written since their last read.

        Returns:
            Tuple of (value, version). Version starts at 0 (no writes).
        """
        with self._lock:
            return self._value, self._version

    @property
    def version(self) -> int:
        """Return the current version (number of writes)."""
        with self._lock:
            return self._version

    def clear(self) -> None:
        """Reset the slot to empty (value=None, version unchanged)."""
        with self._lock:
            self._value = None
