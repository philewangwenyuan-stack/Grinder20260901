"""Runtime coverage checkpoints. Never infer a continuation from robot yaw."""
import math


def coverage_continuation(points, progress_s, regions, default_direction="x", angle_deg=0.0):
    """Find the current/next unfinished coverage edge, skipping turns/connectors.

    All positions are in the source map frame. At a lane end, select the next
    coverage lane; in a lane, retain its direction and projected start position.
    A lap endpoint comes from the old path, so even laps retain their endpoint.
    """
    region_by_id = {str(r.get("region_id", "")): r for r in regions}
    arc = 0.0
    for index in range(len(points) - 1):
        a, b = points[index], points[index + 1]
        dx, dy = float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])
        length = math.hypot(dx, dy)
        end_s = arc + length
        tag = str(a.get("path_type", ""))
        base, sep, lap_text = tag.rpartition("__lap_")
        if not sep:
            base, lap_text = tag, "1"
        region = region_by_id.get(base)
        if (region is not None and tag == str(b.get("path_type", ""))
                and length > 1e-6 and end_s > float(progress_s) + 1e-6):
            configured = str(region.get("global_direction") or default_direction)
            axis = configured.lstrip("-")
            angle = math.radians(float(angle_deg) + (90.0 if axis == "y" else 0.0))
            dot = dx * math.cos(angle) + dy * math.sin(angle)
            # Perpendicular cross-lane turns and diagonal obstacle detours are
            # not coverage headings. Use the next aligned edge in path order.
            if abs(dot) / length >= 0.98:
                fraction = max(0.0, min(1.0, (float(progress_s) - arc) / length))
                last = b
                for candidate in points[index + 2:]:
                    if str(candidate.get("path_type", "")) != tag:
                        break
                    last = candidate
                lap = max(1, int(lap_text))
                # Prefer a later intact lap for full-repeat endpoints when the
                # old path already contains a shortened continuation lap.
                region_points = [p for p in points if str(p.get("path_type", "")).startswith(base + "__lap_")]
                full_tag = str(region_points[-1].get("path_type", "")) if region_points else tag
                full_points = [p for p in points if str(p.get("path_type", "")) == full_tag]
                full_start, full_end = full_points[0], full_points[-1]
                full_lap = int(full_tag.rpartition("__lap_")[2]) if "__lap_" in full_tag else 1
                if full_lap % 2 == 0:
                    full_start, full_end = full_end, full_start
                return {
                    "region_id": base, "lap": lap,
                    "direction": ("-" if dot < 0.0 else "") + axis,
                    "start_pose": {"x": float(a["x"]) + fraction * dx,
                                   "y": float(a["y"]) + fraction * dy},
                    "end_pose": {"x": float(last["x"]), "y": float(last["y"])},
                    "full_start_pose": {"x": float(full_start["x"]), "y": float(full_start["y"])},
                    "full_end_pose": {"x": float(full_end["x"]), "y": float(full_end["y"])},
                }
        arc = end_s
    return None
