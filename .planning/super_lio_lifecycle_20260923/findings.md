# Findings

## Existing behavior

- `super_lio_mode_manager.py` currently waits for service registration and one `/lio/odom` plus one `/map` event for mapping; localization waits for one `/map` event.
- Localization readiness currently becomes true after any `/initialpose` while localizing followed by one `/lio/odom` message.
- Stop currently shuts down the roslaunch parent, waits for known names, kills remaining mapping names, and does not recheck or return a residual list.
- Map save uses `.tmp-{map_id}-{pid}`, validates files, and calls `os.replace`, but does not fsync files/directories. Existing map IDs are immutable because saving refuses an existing bundle.
- APP `RadarRelocalizationRequest` carries pose/covariance only. `RadarRelocalizationStatusResponse` carries `raw_status` and timestamp. `MapCatalogItem` has no map asset revision; `MapPreviewResponse.map_version` is region/edit version.
- Scheduler map save already reports `map_saved_localization_pending`; map saving and localization start are separate operations, but APP feedback is string-based.
- Current `mid360_mapping.launch` includes `super_lio/launch/Livox_mid360.launch`, which starts `super_lio_node` and a transform; the vendor `livox_ros_driver2` driver is launched separately and must remain a single external entry.
- User has pre-existing uncommitted metrics work in `super_lio_mode_manager.py`, `scheduler_node.py`, `sl_linka_adapter.py`, scheduler launch/build/docs, plus a dirty Super-LIO submodule. Do not overwrite or reset those changes.
- A separate active planning folder (`.planning/python-cpp-android-plan`) belongs to an unrelated document task; this implementation has its own planning directory and will not alter the active-plan pointer.
- Super-LIO publishes `/lio/odom` covariance in `ROSWrapper.cpp`; the mode manager currently ignores it. No fitness field was found in the inspected relocation/ROS wrapper API, so a meaningful fitness gate needs a concrete diagnostics source before it can be implemented.
- There is no in-repository Livox driver launch/config or documented fixed UDP port value to safely hard-code. The driver is externally launched; preflight should validate expected ROS publishers/node ownership and expose optional port checks only for explicitly configured managed ports.
- `GetSuperLioStatus.srv` currently contains only a ready boolean, not map revision or residual-node list; existing CMake already generates its services and can be extended.

## Implementation design notes

- Use explicit transient states (`STARTING`, `STOPPING`, and a localization/relocalization state) and terminal `TIMEOUT`/`ERROR`; status must include residual node names and active map identity.
- Track exact launch-owned node registrations/process ownership before cleanup; fixed names alone are insufficient proof of ownership.
- Readiness should use rolling arrival timestamps and message header stamps for streaming topics, validate odometry pose/covariance and quaternion, and use TF lookup. Static `/map` is validated for current map identity/availability rather than requiring ongoing frequency.
- Super-LIO fitness must have an explicit diagnostics source; if unavailable from current odometry, expose a diagnostic signal rather than guessing from pose covariance.
- Generate a stable asset revision from the immutable bundle contents and include it in `map_info.json`, registry, catalog/preview/status, and relocalization requests.
- Protobuf changes are additive with new field numbers; regenerate Python/Android artifacts from the canonical `.proto` and update protocol docs.
