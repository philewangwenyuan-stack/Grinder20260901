# Task continuation direction

Goal: preserve interrupted coverage progress and direction on stop/start and replanning; fresh plans retain configured direction.

- [x] Inspect execution cursor, task identity, map geometry and planner backends.
- [x] Implement guarded runtime continuation, preserve stop cursor, and direction override only for the interrupted region/lap.
- [x] Add offline regression tests and documentation; verify Python 3.8 syntax and existing planner tests.

Completed: Linux Python 3.8, real native C++ planner, 31 tests passed with no skips; Python 3.8 syntax and git diff --check passed. No ROS/hardware testing performed.

Constraints: preserve current staged changes; no hardware actions; no protocol ABI changes. Continuation is runtime-only unless persisted explicitly.

Errors: two exploratory rg paths did not exist; use actual existing paths.
Integration fixture initially searched for a >0.5m edge, while adapter samples every 0.3m; lowered fixture search to 0.1m. Python integration passes.
