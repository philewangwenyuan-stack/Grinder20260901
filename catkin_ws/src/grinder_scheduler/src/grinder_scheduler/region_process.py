"""Work-region recipe identity and immutable execution snapshots (no ROS)."""
import math
import re


def base_region_id(value):
    return re.sub(r"__lap_\d+$", "", str(value or "").strip())


def point_region_id(point):
    if str(point.get("path_type", "")).lower() == "connection":
        return ""
    return base_region_id(point.get("region_id") or point.get("path_type"))


def capture_region_recipes(service, region_ids, max_speed, max_rpm):
    recipes = {}
    for raw_id in region_ids:
        region_id = base_region_id(raw_id)
        if not region_id or region_id in recipes:
            continue
        recipe = service.get_process_settings(region_id)
        if not recipe:
            raise ValueError("Work region {} has no saved process; confirm and save its process settings".format(region_id))
        speed, rpm = float(recipe["run_speed"]), int(recipe["disc_speed_rpm"])
        if not math.isfinite(speed) or not 0 < speed <= max_speed or not 0 <= rpm <= max_rpm:
            raise ValueError("Work region {} has invalid process speed/RPM".format(region_id))
        recipes[region_id] = dict(recipe, run_speed=speed, disc_speed_rpm=rpm)
    return recipes
