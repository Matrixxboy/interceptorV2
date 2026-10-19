"""Structured JSON logging + in-memory ring buffer (§12).

Logs structured JSON with ts_ns, component, event, and additional fields.
Ring buffer is dumped on FAULT for post-mortem analysis.
"""

from __future__ import annotations

import collections
import json
import logging
import sys
import time
from typing import Any


class StructuredFormatter(logging.Formatter):
    """Format log records as structured JSON."""

    def format(self, record: logging.LogRecord) -> str:
        log_entry: dict[str, Any] = {
            "ts_ns": time.monotonic_ns(),
            "wall": self.formatTime(record),
            "level": record.levelname,
            "component": record.name,
            "event": record.getMessage(),
        }
        if record.exc_info and record.exc_info[1]:
            log_entry["exception"] = self.formatException(record.exc_info)

        # Include any extra fields
        for key in ("extra_fields",):
            if hasattr(record, key):
                log_entry.update(getattr(record, key))

        return json.dumps(log_entry, default=str)


class RingBufferHandler(logging.Handler):
    """In-memory ring buffer that stores the last N log records.

    Dumped on FAULT for post-mortem analysis (§12).
    """

    def __init__(self, capacity: int = 1000) -> None:
        super().__init__()
        self.buffer: collections.deque[str] = collections.deque(maxlen=capacity)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
            self.buffer.append(msg)
        except Exception:
            self.handleError(record)

    def dump(self) -> list[str]:
        """Return all buffered log entries."""
        return list(self.buffer)

    def clear(self) -> None:
        """Clear the buffer."""
        self.buffer.clear()


_global_ring_buffer: RingBufferHandler | None = None


def dump_ring_buffer_on_fault() -> list[str]:
    """Dump in-memory ring buffer on FAULT (§12)."""
    global _global_ring_buffer
    if _global_ring_buffer is not None:
        entries = _global_ring_buffer.dump()
        logger = logging.getLogger("fault_dump")
        logger.critical("--- FAULT DUMP: %d log records dumped ---", len(entries))
        return entries
    return []


def setup_logging(level: str = "INFO", json_output: bool = True) -> RingBufferHandler:
    """Configure structured logging for the air-unit.

    Args:
        level: Logging level string.
        json_output: If True, use JSON formatter; otherwise plain text.

    Returns:
        The RingBufferHandler for FAULT dump access.
    """
    global _global_ring_buffer
    root = logging.getLogger()
    root.setLevel(getattr(logging, level.upper(), logging.INFO))

    # Clear existing handlers
    root.handlers.clear()

    # Console handler
    console = logging.StreamHandler(sys.stderr)
    if json_output:
        console.setFormatter(StructuredFormatter())
    else:
        console.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s"))
    root.addHandler(console)

    # Ring buffer handler
    ring = RingBufferHandler(capacity=2000)
    ring.setFormatter(StructuredFormatter())
    root.addHandler(ring)
    _global_ring_buffer = ring

    return ring
