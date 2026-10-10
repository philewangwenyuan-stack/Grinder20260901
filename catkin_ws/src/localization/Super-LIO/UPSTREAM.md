# Super-LIO source provenance

- Upstream: https://github.com/Liansheng-Wang/Super-LIO.git
- Upstream checkout: `239984eb8cff0ba5efdf999e9b90d02b2c7d5c5e`
- License: GPL-3.0, preserved in `LICENSE`; retain third-party notices in the source.
- Imported into the Grinder root repository on 2026-09-28 as ordinary files.

This snapshot includes the existing Grinder/MID-360S integration changes from
the local checkout, including relocation/map services, ROS diagnostics,
configuration, launch files and RViz settings. It is not an unmodified upstream
checkout. Existing tracked files and newly added source/documentation files were
preserved during import; build trees, caches and local analysis outputs are not
distributed as source.
The import also declares the existing `basic` dependency in the Super-LIO
package/export metadata so Catkin dependency selection includes that package.

There is no submodule initialization step. Use Git from the Grinder root for
future changes and compare upstream updates against the checkout above.
The former nested Git history remains in an ignored `.git.nested-backup-*`
directory on the migration workstation; it is not required to build a fresh clone.

## Per-frame localization iteration diagnostics

`/super_lio/registration_diagnostics` retains its existing registration fields
and adds `kf_*` keys to the `super_lio_registration` status. The array header
timestamp is the same frame timestamp as localization odometry. This is
instrumentation only: iteration limits, convergence criteria, state updates and
localization readiness checks are unchanged.

- `kf_iterations`, `kf_max_iterations`, `kf_quit_eps`: actual iteration count,
  configured limit and convergence threshold.
- `kf_convergence_reason`: `threshold_reached`, `iteration_limit`,
  `non_finite_update`, or `no_iterations`. `kf_iteration_limit_reached` means
  the limit was exhausted without convergence; `kf_updates_finite` covers all
  iterations in this frame.
- `kf_iteration_N_update_inf`, `kf_iteration_N_max_component`,
  `kf_iteration_N_max_component_value`, `kf_iteration_N_update_finite`:
  each iteration's maximum absolute update, component name, signed component
  value and finite-value check. N starts at 1. For a non-finite update, the
  maximum is reported as `inf` and the first non-finite component is identified.
- `kf_final_update_inf`, `kf_final_max_component`, and
  `kf_final_{rotation,position,velocity,gyro_bias,accel_bias,gravity}_update_inf`:
  final iteration summary, captured before the filter clears its update vector.
  These are infinity norms (maximum absolute component), not Euclidean norms;
  the six groups have different physical units and must be interpreted separately.

Unconverged frames also emit a warning summary throttled to once per second.
The diagnostic topic retains every frame, including frames omitted by the log
throttle. Numeric values retain double precision to distinguish updates close
to the configured threshold.

After rebuilding Super-LIO in Linux/ROS Noetic and restarting the localization
process through the normal lifecycle, inspect or record the topic:

```bash
rostopic echo /super_lio/registration_diagnostics
rosbag record -O /tmp/super_lio_iteration_trace.bag \
  /initialpose /super_lio/registration_diagnostics /lio/odom /tf /tf_static /rosout
```

Compare unconverged frames with successful neighboring frames. A decreasing
update that remains above the threshold at the iteration limit suggests an
iteration-budget investigation; oscillating or non-finite updates require
examining registration inputs and numerical behavior rather than loosening the
readiness check.
