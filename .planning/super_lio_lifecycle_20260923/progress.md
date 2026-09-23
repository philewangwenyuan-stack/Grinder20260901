# Progress

## 2026-09-23

- Read repository instructions and `planning-with-files` skill.
- Confirmed pre-existing user changes; reviewed targeted diffs and will preserve them.
- Audited current manager, APP protocol, launch ownership, and map-version semantics.
- Confirmed protobuf generation script and service schemas. Map save already reports pending localization in text; map revision is absent from registry/catalog.
- Confirmed Super-LIO `/lio/odom` includes covariance, but no fitness output was found in the inspected public relocation interface.
- No in-repository Livox driver port configuration is available, so fixed port numbers must not be invented.
- Next: inspect registry/test details and local Super-LIO dirty diff; then implement safe ownership/readiness and protocol changes.
