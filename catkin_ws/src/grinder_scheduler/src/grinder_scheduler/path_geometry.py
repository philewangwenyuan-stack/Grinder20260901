"""Sparse global polylines and local sampling. No ROS or hardware dependencies."""
import math


def segment_point_spacing(value):
    """Reject invalid/sub-centimetre sampling rather than allocating huge paths."""
    if isinstance(value, bool):
        raise ValueError("active_segment_point_spacing must be a number, not a boolean")
    spacing = float(value)
    if not math.isfinite(spacing) or spacing < 0.01:
        raise ValueError("active_segment_point_spacing must be finite and >= 0.01 m")
    return spacing


def compact_planner_polyline(points):
    """Remove only forward collinear interior points with identical semantics.

    Preserve reversals, zero-length semantic events, and all actual detours.
    This is geometry-preserving compression, not tolerance-based smoothing.
    """
    result = []
    def semantics(p):
        return tuple(getattr(p, name, None) for name in
                     ("path_type", "point_type", "time", "region_id", "lap_index"))
    for point in points:
        while len(result) >= 2:
            a, b = result[-2:]
            if semantics(a) != semantics(b) or semantics(b) != semantics(point):
                break
            ux, uy = float(b.col - a.col), float(b.row - a.row)
            vx, vy = float(point.col - b.col), float(point.row - b.row)
            # No corner removal across a reversal, stop, or coincident event.
            if ux * vx + uy * vy <= 0.0:
                break
            if abs(ux * vy - uy * vx) > 1e-12 * max(1.0, math.hypot(ux, uy) * math.hypot(vx, vy)):
                break
            result.pop()
        result.append(point)
    return result


def sample_segment(points, spacing):
    """Sample each edge separately, keeping every corner and semantic event.

    Interior samples inherit the outgoing edge's source metadata. Global point
    indices are source references, never new execution/progress indices.
    """
    spacing = segment_point_spacing(spacing)
    if not points:
        return []
    result = [dict(points[0])]
    for a, b in zip(points, points[1:]):
        dx = float(b["x"]) - float(a["x"])
        dy = float(b["y"]) - float(a["y"])
        length = math.hypot(dx, dy)
        # ceil bounds every output edge by spacing, including the last one.
        # Avoid doubling a nominal 0.3 m edge represented as 0.30000000000018.
        count = max(1, int(math.ceil(length / spacing - 1e-10)))
        for step in range(1, count):
            ratio = step / count
            point = dict(a)
            point.update(x=float(a["x"]) + ratio * dx,
                         y=float(a["y"]) + ratio * dy, point_type="segment_sample")
            for key in ("row", "col"):
                if key in a and key in b:
                    point[key] = float(a[key]) + ratio * (float(b[key]) - float(a[key]))
            result.append(point)
        result.append(dict(b))
    return result
