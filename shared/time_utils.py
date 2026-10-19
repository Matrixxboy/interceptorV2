"""Monotonic time utilities.

Rule §2.6: Use monotonic time only for latency/timeouts. Wall-clock only for log filenames.
"""

from __future__ import annotations

import time


def monotonic_ns() -> int:
    """Return monotonic clock in nanoseconds."""
    return time.monotonic_ns()


def monotonic_s() -> float:
    """Return monotonic clock in seconds (float)."""
    return time.monotonic()


def ns_to_ms(ns: int) -> float:
    """Convert nanoseconds to milliseconds."""
    return ns / 1_000_000


def ns_to_s(ns: int) -> float:
    """Convert nanoseconds to seconds."""
    return ns / 1_000_000_000


def elapsed_ms(start_ns: int) -> float:
    """Return milliseconds elapsed since *start_ns* (monotonic)."""
    return ns_to_ms(time.monotonic_ns() - start_ns)


def elapsed_s(start_ns: int) -> float:
    """Return seconds elapsed since *start_ns* (monotonic)."""
    return ns_to_s(time.monotonic_ns() - start_ns)


def wall_clock_iso() -> str:
    """Return wall-clock timestamp in ISO-8601 format (for log filenames only)."""
    return time.strftime("%Y-%m-%dT%H-%M-%S", time.localtime())
