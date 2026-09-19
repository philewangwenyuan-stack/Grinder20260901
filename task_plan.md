# LIVE_MAP save-path repair

## Goal

Ensure `MapSaveRequest(map_id="LIVE_MAP")` exports a new Super-LIO map bundle instead of updating stale registry metadata, and prevent the live-map sentinel from persisting in the saved-map registry.

## Phases

- [complete] Phase 1 — Inspect save, registry migration, ID generation, and existing scheduler test conventions.
- [complete] Phase 2 — Implement defensive registry filtering and live-save ID normalization.
- [complete] Phase 3 — Add regression tests and update protocol/runtime documentation where needed.
- [complete] Phase 4 — Run focused syntax/tests and review the scoped diff.

## Decisions

- `LIVE_MAP` and empty `map_id` identify the live source, never a saved bundle ID.
- Saving still requires active Super-LIO `MAPPING`; the save service remains responsible for switching to localization afterward.
- Existing non-live saved-map behavior is not changed in this repair.

## Errors Encountered

| Error | Resolution |
| --- | --- |
| Previous inspection initially used an obsolete nested SL-LinkA path | Use `third_party/sl_linka/` in the current repository layout. |
| Native Windows focused test could not import `rospy` | Run the ROS-dependent scheduler test in WSL/ROS; keep native `py_compile` as an additional syntax check. |
| Combined unittest process let the live-map test's import stubs mask the real `grinder_scheduler.models` module in a later test | Run import-isolated test modules in separate Python processes. |
| The generated legacy Python protobuf failed with the host's newer C++ protobuf runtime | Force the pure-Python protobuf implementation inside the isolated protocol handler test. |
