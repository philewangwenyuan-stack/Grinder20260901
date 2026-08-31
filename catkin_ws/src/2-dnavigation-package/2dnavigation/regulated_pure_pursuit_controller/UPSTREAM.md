# Upstream provenance

- Repository: https://github.com/datledoan/regulated_pure_pursuit_controller_ros
- Imported commit: `a8dd2e630dbe83f4f76badaf23e723484e6e36d7`
- Imported date: 2026-07-28
- License: Apache-2.0 (see `LICENSE`)

## Local safety fixes

The vendored copy intentionally differs from upstream:

- Treat the `tf2_ros::Buffer` and `Costmap2DROS` pointers supplied by
  `move_base` as non-owning to prevent double deletion.
- Stop and return a collision error when the projected command intersects an
  obstacle.
- Treat poses outside the local costmap as occupied.
- Reject empty or one-pose paths and contain path-transform exceptions.
- Read the odometry topic from the plugin namespace and use the actual
  `move_base` controller frequency for acceleration limiting.
- Own the footprint collision model with `std::unique_ptr`.
