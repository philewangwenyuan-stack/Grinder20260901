# Super-LIO lifecycle and localization quality

## Goal

Implement reliable Super-LIO stop/start/readiness, map-bound relocalization, atomic map bundle saves, and explicit APP protocol status while preserving existing workspace changes.

## Phases

- [complete] Phase 1: map current lifecycle, protocol generation, and tests; preserve pre-existing changes.
- [in_progress] Phase 2: implement manager ownership, stop confirmation, startup health checks, and localization quality gating.
- [pending] Phase 3: add immutable map revision identity and APP protobuf/status fields; regenerate SDKs and document the contract.
- [pending] Phase 4: update focused tests/docs and run minimal static/unit verification available on this host.

## Decisions

- Fitness will be exported from the existing Super-LIO relocation scan matcher as mean squared point-to-plane residual, along with inlier ratio and ESKF convergence, on a stamped diagnostic topic. The localization package already has pre-existing user changes; edits there will be minimal and additive.
- APP-facing map identity uses a SHA-256 `map_revision` over the immutable bundle assets, separate from `MapPreviewResponse.map_version` (region edits).
- Initial poses will enter through a manager service carrying map ID/revision, which validates identity and publishes `/initialpose` atomically. Direct untagged `/initialpose` messages will not satisfy readiness.
- No fixed Livox UDP port numbers will be invented. Managed mode will verify one fresh Livox ROS publisher path and support explicit deployment-configured exclusive ports; actual ports must come from the external Livox driver JSON.

## Constraints

- Keep Python 3.8 compatibility.
- Do not stop or kill unowned ROS processes.
- Preserve Livox driver as an external single startup entry; managed mapping owns Super-LIO/mapper/loop only.
- Save map bundles to unique temporary directories on the target filesystem, fsync files/directories, then atomically publish an immutable revision.
- APP relocalization must supply current map ID and revision; absent or stale identity fails closed.
- Preserve existing user changes in manager, scheduler, launch/config, docs, metrics, and Super-LIO submodule.
- No real hardware or full ROS launch validation from this Windows workspace.

## Errors Encountered

| Error | Resolution |
| --- | --- |
| Initial broad PowerShell reads hit Windows native crash exit codes on some commands | Narrowed reads and kept successful output; no source issue inferred. |
