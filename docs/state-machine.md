# State Machine

See PLAN.MD §6 for the canonical state machine description.

```
INIT ──► IDLE ──► SELECTING ──► FOLLOWING ──► TARGET_LOST ──► IDLE
                                    │              │
                                    └──────────────┘ (re-acquired)

any ──► FAULT (on exception / hw failure)
```
