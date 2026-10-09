# Findings

- Region direction overrides task default. mst27 forces first travel sign from stage.direction.
- Stop currently marks an interrupted lap done; start reprojects globally, losing unambiguous execution cursor.
- Active execution segments include turn and next-lane lead-in; continuation must select the coverage run rather than assume segment heading or robot yaw.
- APP device Start directly sends START. Preview/replan requests also exist, including ad-hoc task_id empty.
- New runtime checkpoint is identity/pose guarded; original path reused under unchanged map stamp, otherwise partial-lap direction supplied through TaskConfigModel.continuation.
- Actual adapter repeated the same mutable PathPoint objects across laps, corrupting previous lap labels; per-lap copies resolve the prerequisite for accurate lap continuation.
