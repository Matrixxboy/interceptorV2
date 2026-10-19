"""Tests for core/latest_slot.py and core/state_machine.py — T1.1 acceptance.

LatestSlot: thread-safety, overwrite semantics, versioning.
StateMachine: exhaustive state × event table, FAULT from anywhere, callbacks.
"""

from __future__ import annotations

import threading

import pytest
from air_unit.core.latest_slot import LatestSlot
from air_unit.core.state_machine import Event, State, StateMachine

# ══════════════════════════════════════════════
# LatestSlot tests
# ══════════════════════════════════════════════


class TestLatestSlot:
    """LatestSlot — overwrite semantics, thread safety."""

    def test_empty_slot_returns_none(self) -> None:
        slot: LatestSlot[int] = LatestSlot()
        assert slot.get() is None

    def test_put_and_get(self) -> None:
        slot: LatestSlot[str] = LatestSlot()
        slot.put("hello")
        assert slot.get() == "hello"

    def test_overwrite_semantics(self) -> None:
        slot: LatestSlot[int] = LatestSlot()
        slot.put(1)
        slot.put(2)
        slot.put(3)
        assert slot.get() == 3

    def test_versioning(self) -> None:
        slot: LatestSlot[int] = LatestSlot()
        assert slot.version == 0
        slot.put(10)
        assert slot.version == 1
        slot.put(20)
        assert slot.version == 2
        val, ver = slot.get_versioned()
        assert val == 20
        assert ver == 2

    def test_clear(self) -> None:
        slot: LatestSlot[int] = LatestSlot()
        slot.put(42)
        slot.clear()
        assert slot.get() is None
        # version is NOT reset by clear
        assert slot.version == 1

    def test_thread_safety(self) -> None:
        """Multiple writers, one reader — no crashes, reader gets a valid value."""
        slot: LatestSlot[int] = LatestSlot()
        n_writes = 1000

        def writer(start: int) -> None:
            for i in range(start, start + n_writes):
                slot.put(i)

        threads = [threading.Thread(target=writer, args=(i * n_writes,)) for i in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        val = slot.get()
        assert val is not None
        assert slot.version == 4 * n_writes


# ══════════════════════════════════════════════
# StateMachine tests — exhaustive state × event
# ══════════════════════════════════════════════


class TestStateMachineTransitions:
    """Exhaustive transition table tests (§6)."""

    def test_init_to_idle(self) -> None:
        sm = StateMachine()
        assert sm.state == State.INIT
        sm.handle_event(Event.SYSTEM_READY)
        assert sm.state == State.IDLE

    def test_idle_to_selecting(self) -> None:
        sm = StateMachine(state=State.IDLE)
        sm.handle_event(Event.LOCK_REQUEST)
        assert sm.state == State.SELECTING

    def test_selecting_to_following(self) -> None:
        sm = StateMachine(state=State.SELECTING)
        sm.handle_event(Event.TRACK_FOUND)
        assert sm.state == State.FOLLOWING

    def test_selecting_timeout_to_idle(self) -> None:
        sm = StateMachine(state=State.SELECTING)
        sm.handle_event(Event.LOCK_TIMEOUT)
        assert sm.state == State.IDLE

    def test_following_to_target_lost(self) -> None:
        sm = StateMachine(state=State.FOLLOWING)
        sm.handle_event(Event.TARGET_MISSING)
        assert sm.state == State.TARGET_LOST

    def test_following_unlock_to_idle(self) -> None:
        sm = StateMachine(state=State.FOLLOWING)
        sm.handle_event(Event.UNLOCK_REQUEST)
        assert sm.state == State.IDLE

    def test_following_mode_not_guided_to_idle(self) -> None:
        sm = StateMachine(state=State.FOLLOWING)
        sm.handle_event(Event.MODE_NOT_GUIDED)
        assert sm.state == State.IDLE

    def test_following_gcs_timeout_to_idle(self) -> None:
        sm = StateMachine(state=State.FOLLOWING)
        sm.handle_event(Event.GCS_TIMEOUT)
        assert sm.state == State.IDLE

    def test_target_lost_reacquired(self) -> None:
        sm = StateMachine(state=State.TARGET_LOST)
        sm.handle_event(Event.TARGET_REACQUIRED)
        assert sm.state == State.FOLLOWING

    def test_target_lost_timeout_to_idle(self) -> None:
        sm = StateMachine(state=State.TARGET_LOST)
        sm.handle_event(Event.TARGET_LOST_TIMEOUT)
        assert sm.state == State.IDLE

    def test_target_lost_unlock_to_idle(self) -> None:
        sm = StateMachine(state=State.TARGET_LOST)
        sm.handle_event(Event.UNLOCK_REQUEST)
        assert sm.state == State.IDLE

    def test_target_lost_mode_not_guided_to_idle(self) -> None:
        sm = StateMachine(state=State.TARGET_LOST)
        sm.handle_event(Event.MODE_NOT_GUIDED)
        assert sm.state == State.IDLE

    def test_target_lost_gcs_timeout_to_idle(self) -> None:
        sm = StateMachine(state=State.TARGET_LOST)
        sm.handle_event(Event.GCS_TIMEOUT)
        assert sm.state == State.IDLE


class TestStateMachineFault:
    """FAULT_DETECTED transitions to FAULT from any state (S7)."""

    @pytest.mark.parametrize(
        "initial_state",
        [State.INIT, State.IDLE, State.SELECTING, State.FOLLOWING, State.TARGET_LOST],
    )
    def test_fault_from_any_state(self, initial_state: State) -> None:
        sm = StateMachine(state=initial_state)
        sm.handle_event(Event.FAULT_DETECTED)
        assert sm.state == State.FAULT

    def test_fault_from_fault_stays_fault(self) -> None:
        sm = StateMachine(state=State.FAULT)
        sm.handle_event(Event.FAULT_DETECTED)
        assert sm.state == State.FAULT


class TestStateMachineNoTransition:
    """Undefined (state, event) pairs stay in current state."""

    def test_idle_ignores_track_found(self) -> None:
        sm = StateMachine(state=State.IDLE)
        sm.handle_event(Event.TRACK_FOUND)
        assert sm.state == State.IDLE

    def test_init_ignores_lock_request(self) -> None:
        sm = StateMachine(state=State.INIT)
        sm.handle_event(Event.LOCK_REQUEST)
        assert sm.state == State.INIT

    def test_following_ignores_system_ready(self) -> None:
        sm = StateMachine(state=State.FOLLOWING)
        sm.handle_event(Event.SYSTEM_READY)
        assert sm.state == State.FOLLOWING


class TestStateMachineCallbacks:
    """Transition callbacks are fired correctly."""

    def test_callback_fires_on_transition(self) -> None:
        sm = StateMachine(state=State.IDLE)
        calls: list[tuple[State, Event, State]] = []
        sm.on_transition(lambda old, ev, new: calls.append((old, ev, new)))

        sm.handle_event(Event.LOCK_REQUEST)
        assert len(calls) == 1
        assert calls[0] == (State.IDLE, Event.LOCK_REQUEST, State.SELECTING)

    def test_callback_not_fired_on_no_transition(self) -> None:
        sm = StateMachine(state=State.IDLE)
        calls: list[tuple[State, Event, State]] = []
        sm.on_transition(lambda old, ev, new: calls.append((old, ev, new)))

        sm.handle_event(Event.TRACK_FOUND)  # no transition defined
        assert len(calls) == 0

    def test_callback_fires_on_fault(self) -> None:
        sm = StateMachine(state=State.FOLLOWING)
        calls: list[tuple[State, Event, State]] = []
        sm.on_transition(lambda old, ev, new: calls.append((old, ev, new)))

        sm.handle_event(Event.FAULT_DETECTED)
        assert len(calls) == 1
        assert calls[0][2] == State.FAULT


class TestStateMachineHelpers:
    """Helper methods."""

    def test_is_following(self) -> None:
        sm = StateMachine(state=State.FOLLOWING)
        assert sm.is_following()
        sm.state = State.IDLE
        assert not sm.is_following()

    def test_is_fault(self) -> None:
        sm = StateMachine(state=State.FAULT)
        assert sm.is_fault()

    def test_reset(self) -> None:
        sm = StateMachine(state=State.FOLLOWING)
        sm.reset()
        assert sm.state == State.INIT


class TestStateMachineExhaustive:
    """Verify every (State, Event) pair is handled without error."""

    @pytest.mark.parametrize("state", list(State))
    @pytest.mark.parametrize("event", list(Event))
    def test_no_crash(self, state: State, event: Event) -> None:
        sm = StateMachine(state=state)
        result = sm.handle_event(event)
        assert isinstance(result, State)
