"""Data-driven state machine (§6, T1.1).

States: INIT → IDLE → SELECTING → FOLLOWING → TARGET_LOST → IDLE, plus FAULT (from anywhere).
Transition table is data-driven and unit tested exhaustively (every state × event).
"""

from __future__ import annotations

import enum
import logging
from collections.abc import Callable
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


class State(enum.Enum):
    """Air-unit state machine states (§6)."""

    INIT = "INIT"
    IDLE = "IDLE"
    SELECTING = "SELECTING"
    FOLLOWING = "FOLLOWING"
    TARGET_LOST = "TARGET_LOST"
    FAULT = "FAULT"


class Event(enum.Enum):
    """Events that trigger state transitions (§6)."""

    SYSTEM_READY = "SYSTEM_READY"  # camera+FC+NPU healthy
    LOCK_REQUEST = "LOCK_REQUEST"  # pilot tapped, FC in GUIDED
    TRACK_FOUND = "TRACK_FOUND"  # track found within LOCK_RADIUS_NORM
    LOCK_TIMEOUT = "LOCK_TIMEOUT"  # no match in 1 s
    TARGET_MISSING = "TARGET_MISSING"  # track missing > TARGET_LOST_TIMEOUT_S
    TARGET_REACQUIRED = "TARGET_REACQUIRED"  # re-acquired (same ID or gate match)
    TARGET_LOST_TIMEOUT = "TARGET_LOST_TIMEOUT"  # timeout TARGET_LOST_RTL_S
    UNLOCK_REQUEST = "UNLOCK_REQUEST"  # pilot requested unlock
    MODE_NOT_GUIDED = "MODE_NOT_GUIDED"  # FC mode ≠ GUIDED
    GCS_TIMEOUT = "GCS_TIMEOUT"  # GCS heartbeat timeout
    FAULT_DETECTED = "FAULT_DETECTED"  # exception / hw failure


@dataclass
class TransitionResult:
    """Result of a state transition."""

    new_state: State
    action: str  # human-readable action description


# Callback type for transition side-effects
TransitionCallback = Callable[[State, Event, State], None]


@dataclass
class StateMachine:
    """Data-driven state machine with exhaustive transition table (§6).

    The transition table is defined as a dict mapping (State, Event) → TransitionResult.
    Any (state, event) pair not in the table is treated as "no transition" (stay in current state),
    EXCEPT that FAULT_DETECTED always transitions to FAULT from any state.
    """

    state: State = State.INIT
    _callbacks: list[TransitionCallback] = field(default_factory=list)

    # ── Transition table (§6) ───────────────
    TRANSITIONS: dict[tuple[State, Event], TransitionResult] = field(
        default_factory=lambda: {
            # INIT
            (State.INIT, Event.SYSTEM_READY): TransitionResult(State.IDLE, "start telemetry"),
            # IDLE
            (State.IDLE, Event.LOCK_REQUEST): TransitionResult(State.SELECTING, "map tap → track"),
            # SELECTING
            (State.SELECTING, Event.TRACK_FOUND): TransitionResult(
                State.FOLLOWING, "init PIDs, reset filters"
            ),
            (State.SELECTING, Event.LOCK_TIMEOUT): TransitionResult(
                State.IDLE, "notify GCS lock_failed"
            ),
            # FOLLOWING
            (State.FOLLOWING, Event.TARGET_MISSING): TransitionResult(
                State.TARGET_LOST, "zero setpoints"
            ),
            (State.FOLLOWING, Event.UNLOCK_REQUEST): TransitionResult(State.IDLE, "stop output"),
            (State.FOLLOWING, Event.MODE_NOT_GUIDED): TransitionResult(
                State.IDLE, "stop output (pilot took over)"
            ),
            (State.FOLLOWING, Event.GCS_TIMEOUT): TransitionResult(State.IDLE, "hover per S5"),
            # TARGET_LOST
            (State.TARGET_LOST, Event.TARGET_REACQUIRED): TransitionResult(
                State.FOLLOWING, "resume following"
            ),
            (State.TARGET_LOST, Event.TARGET_LOST_TIMEOUT): TransitionResult(
                State.IDLE, "request LOITER"
            ),
            (State.TARGET_LOST, Event.UNLOCK_REQUEST): TransitionResult(State.IDLE, "stop output"),
            (State.TARGET_LOST, Event.MODE_NOT_GUIDED): TransitionResult(
                State.IDLE, "stop output (pilot took over)"
            ),
            (State.TARGET_LOST, Event.GCS_TIMEOUT): TransitionResult(State.IDLE, "hover per S5"),
        }
    )

    def on_transition(self, callback: TransitionCallback) -> None:
        """Register a callback invoked on every state transition.

        Args:
            callback: Function(old_state, event, new_state) called after transition.
        """
        self._callbacks.append(callback)

    def handle_event(self, event: Event) -> State:
        """Process an event and transition to the next state.

        FAULT_DETECTED always transitions to FAULT from any state (§6).
        Unknown (state, event) pairs are silently ignored (no transition).

        Args:
            event: The event to process.

        Returns:
            The new state after processing.
        """
        old_state = self.state

        # FAULT_DETECTED: always go to FAULT from any state (S7)
        if event == Event.FAULT_DETECTED:
            self.state = State.FAULT
            action = "zero setpoints, alert GCS"
            logger.error("FAULT: %s → %s (action: %s)", old_state.value, self.state.value, action)
            self._fire_callbacks(old_state, event, self.state)
            return self.state

        key = (self.state, event)
        result = self.TRANSITIONS.get(key)

        if result is None:
            # No transition defined — stay in current state
            logger.debug(
                "No transition for (%s, %s) — staying in %s",
                self.state.value,
                event.value,
                self.state.value,
            )
            return self.state

        self.state = result.new_state
        logger.info(
            "Transition: %s -[%s]→ %s (action: %s)",
            old_state.value,
            event.value,
            self.state.value,
            result.action,
        )
        self._fire_callbacks(old_state, event, self.state)
        return self.state

    # Alias
    handle = handle_event

    def _fire_callbacks(self, old: State, event: Event, new: State) -> None:
        """Fire all registered transition callbacks."""
        for cb in self._callbacks:
            try:
                cb(old, event, new)
            except Exception:
                logger.exception("Transition callback failed")

    def is_following(self) -> bool:
        """Return True if currently in FOLLOWING state."""
        return self.state == State.FOLLOWING

    def is_fault(self) -> bool:
        """Return True if currently in FAULT state."""
        return self.state == State.FAULT

    def reset(self) -> None:
        """Reset state machine to INIT."""
        self.state = State.INIT
