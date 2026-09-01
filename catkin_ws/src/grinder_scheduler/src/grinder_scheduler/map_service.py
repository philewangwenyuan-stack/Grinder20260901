import base64
import json
import math
import os
import threading
import time

import cv2
import numpy as np
try:
    import rospy  # type: ignore
except Exception:
    rospy = None

from grinder_scheduler.models import MapSnapshot, normalize_planning_direction


UNKNOWN_MASK_VALUE = 32767


class MapService:
    def __init__(self):
        self._lock = threading.Lock()
        self._raw_msg = None
        self._overlay = None
        self._pending_overlay = None
        self._map_version = 0
        self._work_regions = {}
        self._obstacle_regions = {}
        self._erase_regions = {}
        self._crop_region = None
        self._last_edit_message = ""
        self._draw_region_id_on_preview = True
        self._draw_region_label_on_preview = True
        self._preview_rga_enabled = False
        self._preview_rga_backend = "auto"
        self._preview_rga_available = False
        self._preview_rga_fallback_warned = False

    def set_draw_region_id_on_preview(self, enabled):
        self._draw_region_id_on_preview = bool(enabled)

    def set_draw_region_label_on_preview(self, enabled):
        self._draw_region_label_on_preview = bool(enabled)

    def configure_preview_rga(self, enabled=False, backend="auto", available=False):
        self._preview_rga_enabled = bool(enabled)
        self._preview_rga_backend = str(backend or "auto").strip().lower() or "auto"
        self._preview_rga_available = bool(available)
        self._preview_rga_fallback_warned = False

    def _next_order_index(self, region_map):
        if not region_map:
            return 10
        return max(int(item.get("order_index", 0)) for item in region_map.values()) + 10

    def _generate_region_id(self, prefix, region_map):
        index = 1
        while True:
            candidate = "{}_{}".format(prefix, index)
            if candidate not in region_map:
                return candidate
            index += 1

    def _normalize_target_type(self, target_region_type):
        text = str(target_region_type).strip().upper()
        if text in ("WORK", "REGION_TYPE_WORK", "1"):
            return "WORK"
        if text in ("OBSTACLE", "REGION_TYPE_OBSTACLE", "2"):
            return "OBSTACLE"
        if text in ("ERASE", "REGION_TYPE_ERASE", "3"):
            return "ERASE"
        if text in ("CROP", "REGION_TYPE_CROP", "4"):
            return "CROP"
        return "UNKNOWN"

    def _normalize_rect_points(self, points):
        if not points:
            return []
        xs = [float(item.get("x", 0.0)) for item in points]
        ys = [float(item.get("y", 0.0)) for item in points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        width = max_x - min_x
        height = max_y - min_y
        if width <= 1e-6:
            max_x = min_x + 0.1
        if height <= 1e-6:
            max_y = min_y + 0.1
        return [
            {"x": min_x, "y": min_y},
            {"x": max_x, "y": min_y},
            {"x": max_x, "y": max_y},
            {"x": min_x, "y": max_y},
        ]

    def _default_region_start_end(self, points):
        if not points:
            return {"x": 0.0, "y": 0.0, "heading_deg": 0.0}, {"x": 0.0, "y": 0.0, "heading_deg": 0.0}
        xs = [float(item.get("x", 0.0)) for item in points]
        ys = [float(item.get("y", 0.0)) for item in points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        # 默认规则：左上角起点，右下角终点
        return (
            {"x": min_x, "y": max_y, "heading_deg": 0.0},
            {"x": max_x, "y": min_y, "heading_deg": 0.0},
        )

    def _upsert_region(self, region_map, region, default_name, default_prefix, default_type):
        name = (region.get("name") or default_name or default_prefix).strip()
        region_id = (region.get("region_id") or region.get("id") or "").strip()
        if not region_id:
            for existing_id, existing in region_map.items():
                if existing.get("name", "").strip() == name and name:
                    region_id = existing_id
                    break
        if not region_id:
            region_id = self._generate_region_id(default_prefix, region_map)

        existing = region_map.get(region_id, {})
        points = region.get("points")
        if points is None:
            points = existing.get("points", [])

        order_index = region.get("order_index")
        if order_index is None:
            order_index = region.get("priority")
        if order_index is None:
            order_index = existing.get("order_index")
        if order_index is None:
            order_index = self._next_order_index(region_map)

        normalized = {
            "name": name or existing.get("name", default_name),
            "points": points,
            "region_id": region_id,
            "order_index": int(order_index),
            "enabled": bool(region.get("enabled", existing.get("enabled", True))),
            "color_argb": int(region.get("color_argb", existing.get("color_argb", 0))),
            "closed": bool(region.get("closed", existing.get("closed", True))),
            "region_type": int(region.get("region_type", existing.get("region_type", default_type))),
        }
        region_direction = normalize_planning_direction(
            region.get("global_direction", ""),
            existing.get("global_direction", "x"),
        )
        normalized["global_direction"] = region_direction
        default_start, default_end = self._default_region_start_end(points)
        start_pose = region.get("start_pose", existing.get("start_pose", {})) or {}
        end_pose = region.get("end_pose", existing.get("end_pose", {})) or {}
        normalized["start_pose"] = {
            "x": float(start_pose.get("x", default_start["x"])),
            "y": float(start_pose.get("y", default_start["y"])),
            "heading_deg": float(start_pose.get("heading_deg", 0.0)),
        }
        normalized["end_pose"] = {
            "x": float(end_pose.get("x", default_end["x"])),
            "y": float(end_pose.get("y", default_end["y"])),
            "heading_deg": float(end_pose.get("heading_deg", 0.0)),
        }
        previous_points = existing.get("points", [])
        region_map[region_id] = normalized
        return normalized, previous_points

    def _delete_region(self, command):
        target_id = (command.get("target_region_id") or command.get("region_id") or "").strip()
        target_name = (command.get("region_name") or "").strip()
        target_type = self._normalize_target_type(command.get("target_region_type", ""))

        removed_regions = []

        def _delete_from(region_map):
            if target_id and target_id in region_map:
                removed = region_map.pop(target_id)
                removed_regions.append(removed)
                return True
            if target_name:
                for key, item in list(region_map.items()):
                    if item.get("name", "").strip() == target_name:
                        removed = region_map.pop(key)
                        removed_regions.append(removed)
                        return True
            return False

        if target_type == "WORK":
            _delete_from(self._work_regions)
        elif target_type == "OBSTACLE":
            _delete_from(self._obstacle_regions)
        elif target_type == "ERASE":
            _delete_from(self._erase_regions)
        elif target_type == "CROP":
            # Crop region is singleton by design, delete it unconditionally for robustness.
            if self._crop_region is not None:
                removed_regions.append(self._crop_region)
                self._crop_region = None
        else:
            # Unknown type: try both maps.
            found = _delete_from(self._work_regions)
            if not found:
                found_obs = _delete_from(self._obstacle_regions)
                found_erase = False if found_obs else _delete_from(self._erase_regions)
                if (not found_obs) and (not found_erase) and (self._crop_region is not None):
                    removed_regions.append(self._crop_region)
                    self._crop_region = None
        return removed_regions

    def set_raw_map(self, map_msg):
        width = int(getattr(map_msg.info, "width", 0) or 0)
        height = int(getattr(map_msg.info, "height", 0) or 0)
        expected_cells = width * height
        actual_cells = len(getattr(map_msg, "data", []) or [])
        if width <= 0 or height <= 0 or actual_cells != expected_cells:
            if rospy is not None:
                rospy.logwarn_throttle(
                    1.0,
                    "MapService rejected incomplete raw map: size=%dx%d expected_cells=%d actual_cells=%d",
                    width,
                    height,
                    expected_cells,
                    actual_cells,
                )
            return False
        with self._lock:
            self._raw_msg = map_msg
            shape = (map_msg.info.height, map_msg.info.width)
            if self._overlay is None or self._overlay.shape != shape:
                if self._pending_overlay is not None and self._pending_overlay.shape == shape:
                    self._overlay = self._pending_overlay.astype(np.int16)
                    self._pending_overlay = None
                else:
                    self._overlay = np.full(shape, UNKNOWN_MASK_VALUE, dtype=np.int16)
        return True

    def has_map(self):
        with self._lock:
            return self._raw_msg is not None

    def get_overlay_regions(self):
        with self._lock:
            return self._serialize_regions()

    def get_map_version(self):
        with self._lock:
            return int(self._map_version)

    def get_map_info(self):
        with self._lock:
            if self._raw_msg is None:
                return None
            info = self._raw_msg.info
            header = self._raw_msg.header
            stamp_ns = int(getattr(header.stamp, "secs", 0)) * 1000000000 + int(getattr(header.stamp, "nsecs", 0))
            return {
                "frame_id": self._raw_msg.header.frame_id,
                "seq": int(getattr(header, "seq", 0)),
                "stamp_ns": stamp_ns,
                "width": info.width,
                "height": info.height,
                "resolution": info.resolution,
                "origin_x": info.origin.position.x,
                "origin_y": info.origin.position.y,
                "map_version": self._map_version,
            }

    def compose_map(self):
        with self._lock:
            if self._raw_msg is None:
                return None
            return self._compose_grid_locked()

    def _compose_grid_locked(self):
        raw = np.array(self._raw_msg.data, dtype=np.int16).reshape((self._raw_msg.info.height, self._raw_msg.info.width))
        overlay = self._overlay.copy()
        for value in (0, 100, -1):
            mask = overlay == value
            raw[mask] = value
        # Enforce region semantics on every compose to avoid stale overlay state.
        # Apply erase first; obstacle wins on overlap so a safety exclusion cannot
        # be accidentally reopened by an older erase region.
        obstacle_regions = sorted(self._obstacle_regions.values(), key=lambda item: int(item.get("order_index", 0)))
        erase_regions = sorted(self._erase_regions.values(), key=lambda item: int(item.get("order_index", 0)))
        for region in erase_regions:
            if not bool(region.get("enabled", True)):
                continue
            points = region.get("points", []) or []
            if len(points) < 3:
                continue
            self._fill_polygon_on_grid(raw, points, 0)
        for region in obstacle_regions:
            if not bool(region.get("enabled", True)):
                continue
            points = region.get("points", []) or []
            if len(points) < 3:
                continue
            self._fill_polygon_on_grid(raw, points, 100)
        if self._crop_region is not None and bool(self._crop_region.get("enabled", True)):
            points = self._crop_region.get("points", []) or []
            if len(points) >= 3:
                crop_mask = np.zeros(raw.shape, dtype=np.uint8)
                polygon = np.array(
                    [[self._world_to_grid(pt["x"], pt["y"])[1], self._world_to_grid(pt["x"], pt["y"])[0]] for pt in points],
                    dtype=np.int32,
                )
                cv2.fillPoly(crop_mask, [polygon], 255)
                raw[crop_mask == 0] = -1
        return raw

    def _fill_polygon_on_grid(self, grid, world_points, value):
        if grid is None or not world_points:
            return
        polygon = np.array(
            [[self._world_to_grid(pt["x"], pt["y"])[1], self._world_to_grid(pt["x"], pt["y"])[0]] for pt in world_points],
            dtype=np.int32,
        )
        cv2.fillPoly(grid, [polygon], int(value))

    def apply_edit(self, command):
        with self._lock:
            if self._raw_msg is None:
                return False, "Raw map not available", self._map_version

            operation = command.get("operation", "")
            if rospy is not None:
                region = command.get("region", {}) or {}
                points = region.get("points", []) or []
                polygon = command.get("polygon", []) or []
                rospy.loginfo(
                    "MapService.apply_edit begin: op=%s map_version=%d region_id=%s region_type=%s region_points=%d polygon_points=%d target_region_id=%s",
                    operation,
                    int(self._map_version),
                    str(region.get("region_id", "")).strip() or "<empty>",
                    str(region.get("region_type", "")),
                    len(points),
                    len(polygon),
                    str(command.get("target_region_id", "")).strip() or "<empty>",
                )
            if operation == "UPSERT_WORK_REGION":
                region = command.get("region", {})
                self._upsert_region(
                    self._work_regions,
                    region,
                    command.get("region_name") or "work_region",
                    "work_region",
                    1,
                )
            elif operation in ("UPSERT_OBSTACLE_REGION", "UPSERT_ERASE_REGION", "UPSERT_CROP_REGION"):
                region = dict(command.get("region", {}) or {})
                if operation == "UPSERT_OBSTACLE_REGION":
                    region["region_type"] = 2
                    default_name = command.get("region_name") or "obstacle_region"
                    default_prefix = "obstacle_region"
                elif operation == "UPSERT_ERASE_REGION":
                    region["region_type"] = 3
                    default_name = command.get("region_name") or "erase_region"
                    default_prefix = "erase_region"
                else:
                    region["region_type"] = 4
                    # Crop region is singleton by design.
                    region["region_id"] = "crop_region_1"
                    if not str(region.get("name", "")).strip():
                        region["name"] = command.get("region_name") or "crop_region"
                    default_name = command.get("region_name") or "crop_region"
                    default_prefix = "crop_region"
                if operation == "UPSERT_OBSTACLE_REGION":
                    target_regions = self._obstacle_regions
                    default_type = 2
                elif operation == "UPSERT_ERASE_REGION":
                    target_regions = self._erase_regions
                    default_type = 3
                else:
                    target_regions = (
                        {"crop_region_1": self._crop_region}
                        if self._crop_region is not None
                        else {}
                    )
                    default_type = 4
                normalized, previous_points = self._upsert_region(
                    target_regions,
                    region,
                    default_name,
                    default_prefix,
                    default_type,
                )
                if previous_points:
                    self._fill_polygon(previous_points, UNKNOWN_MASK_VALUE)
                region_type = int(normalized.get("region_type", 2))
                if region_type == 3:
                    # Erase-region: clear obstacles and heal small gaps in straight lines.
                    self._apply_erase_region_patch(normalized.get("points", []), command.get("brush_radius", 0.2))
                elif region_type == 4:
                    # Crop-region: singleton rectangular region controlling usable map area.
                    crop_points = self._normalize_rect_points(normalized.get("points", []))
                    normalized["points"] = crop_points
                    normalized["region_id"] = "crop_region_1"
                    normalized["name"] = normalized.get("name", "crop_region") or "crop_region"
                    self._crop_region = normalized
                else:
                    # Obstacle-region: update composed-map obstacle immediately.
                    # If region is disabled, keep this area cleared in overlay.
                    if bool(normalized.get("enabled", True)):
                        self._fill_polygon(normalized.get("points", []), 100)
                    else:
                        self._fill_polygon(normalized.get("points", []), UNKNOWN_MASK_VALUE)
            elif operation == "DELETE_REGION":
                removed_regions = self._delete_region(command)
                if not removed_regions:
                    return False, "Region not found for delete", self._map_version
                for removed in removed_regions:
                    # Region semantics are reapplied during compose. Clear the
                    # removed region's persistent raster patch first.
                    if int(removed.get("region_type", 0)) in (2, 3):
                        self._fill_polygon(removed.get("points", []), UNKNOWN_MASK_VALUE)
            elif operation in ("PAINT_FREE", "PAINT_OCCUPIED", "PAINT_UNKNOWN"):
                value_map = {
                    "PAINT_FREE": 0,
                    "PAINT_OCCUPIED": 100,
                    "PAINT_UNKNOWN": -1,
                }
                self._paint_patch(command.get("polygon", []), command.get("brush_radius", 0.2), value_map[operation])
                if operation == "PAINT_FREE":
                    # Clear patch should also smooth narrow breaks for planning stability.
                    self._apply_erase_region_patch(command.get("polygon", []), command.get("brush_radius", 0.2))
            elif operation == "CLEAR_OVERLAY_PATCH":
                self._paint_patch(command.get("polygon", []), command.get("brush_radius", 0.2), UNKNOWN_MASK_VALUE)
            else:
                return False, "Unsupported map edit operation", self._map_version

            self._map_version += 1
            self._last_edit_message = operation
            if rospy is not None:
                rospy.loginfo(
                    "MapService.apply_edit done: op=%s map_version=%d work_regions=%d obstacle_regions=%d erase_regions=%d crop_region=%s",
                    operation,
                    int(self._map_version),
                    len(self._work_regions),
                    len(self._obstacle_regions),
                    len(self._erase_regions),
                    "yes" if self._crop_region is not None else "no",
                )
            return True, "Map overlay updated", self._map_version

    def create_preview(
        self,
        pose,
        max_edge=512,
        image_format="jpg",
        include_overlay=True,
        alignment_yaw=None,
        aligned_frame_id=None,
    ):
        with self._lock:
            if self._raw_msg is None:
                raise RuntimeError("Raw map not available")
            info = self._raw_msg.info
            composed = self._compose_grid_locked()

            width = int(info.width)
            height = int(info.height)
            resolution = float(info.resolution)
            origin_x = float(info.origin.position.x)
            origin_y = float(info.origin.position.y)
            frame_id = self._raw_msg.header.frame_id
            has_alignment = alignment_yaw is not None
            display_yaw = self._sanitize_alignment_yaw(alignment_yaw)
            display_grid = composed
            if display_yaw is not None:
                display_grid, origin_x, origin_y = self._rotate_grid_to_aligned_frame(
                    composed, origin_x, origin_y, resolution, display_yaw
                )
                height, width = display_grid.shape[:2]
            if has_alignment:
                frame_id = str(aligned_frame_id or frame_id)

            preview = self._build_preview_image(
                display_grid,
                pose,
                max_edge=max_edge,
                include_overlay=include_overlay,
                origin_x=origin_x,
                origin_y=origin_y,
                resolution=resolution,
                width=width,
                height=height,
                display_yaw=display_yaw,
            )
            overlay_json = self._build_overlay_json(
                preview.shape[1],
                preview.shape[0],
                width,
                height,
                origin_x,
                origin_y,
                resolution,
                frame_id,
                display_yaw,
                self._pose_to_display_pose(pose, display_yaw),
            )
            encode_ext = ".png" if image_format.lower() == "png" else ".jpg"
            ok, buffer = cv2.imencode(encode_ext, preview)
            if not ok:
                raise RuntimeError("Failed to encode map preview image")
            return MapSnapshot(
                map_version=self._map_version,
                frame_id=frame_id,
                resolution=resolution,
                width=width,
                height=height,
                origin_x=origin_x,
                origin_y=origin_y,
                preview_data=buffer.tobytes(),
                preview_format=encode_ext.lstrip("."),
                overlay_json=overlay_json,
                preview_image=preview.copy(),
            )

    def _build_preview_image(
        self,
        display_grid,
        pose,
        max_edge,
        include_overlay,
        origin_x,
        origin_y,
        resolution,
        width,
        height,
        display_yaw,
    ):
        image = self._occupancy_to_bgr(display_grid)
        if include_overlay:
            self._draw_regions(image, origin_x, origin_y, resolution, width, height, display_yaw)
        self._draw_pose(image, pose, origin_x, origin_y, resolution, width, height, display_yaw)
        return self._resize_with_aspect(image, max_edge=max_edge)

    def _paint_patch(self, polygon_points, brush_radius, value):
        if self._overlay is None:
            return
        if polygon_points:
            if len(polygon_points) == 1:
                row, col = self._world_to_grid(polygon_points[0]["x"], polygon_points[0]["y"])
                radius_px = max(1, int(round(brush_radius / max(self._raw_msg.info.resolution, 1e-6))))
                cv2.circle(self._overlay, (col, row), radius_px, int(value), thickness=-1)
            else:
                self._fill_polygon(polygon_points, value)

    def _build_patch_mask(self, polygon_points, brush_radius):
        if self._overlay is None or self._raw_msg is None:
            return None
        mask = np.zeros(self._overlay.shape, dtype=np.uint8)
        if not polygon_points:
            return mask
        if len(polygon_points) == 1:
            row, col = self._world_to_grid(polygon_points[0]["x"], polygon_points[0]["y"])
            radius_px = max(1, int(round(float(brush_radius) / max(self._raw_msg.info.resolution, 1e-6))))
            cv2.circle(mask, (col, row), radius_px, 255, thickness=-1)
            return mask
        polygon = np.array(
            [[self._world_to_grid(pt["x"], pt["y"])[1], self._world_to_grid(pt["x"], pt["y"])[0]] for pt in polygon_points],
            dtype=np.int32,
        )
        cv2.fillPoly(mask, [polygon], 255)
        return mask

    def _apply_erase_region_patch(self, polygon_points, brush_radius):
        if self._overlay is None or self._raw_msg is None:
            return
        patch_mask = self._build_patch_mask(polygon_points, brush_radius)
        if patch_mask is None or int(np.count_nonzero(patch_mask)) == 0:
            return

        # Step 1: explicit clear inside erase patch.
        self._overlay[patch_mask > 0] = 0

        # Step 2: linear gap heal (horizontal + vertical close) inside patch only.
        raw = np.array(self._raw_msg.data, dtype=np.int16).reshape((self._raw_msg.info.height, self._raw_msg.info.width))
        composed = raw.copy()
        for value in (0, 100, -1):
            mask = self._overlay == value
            composed[mask] = value
        free_mask = np.where(composed == 0, 255, 0).astype(np.uint8)

        radius_px = max(1, int(round(float(brush_radius) / max(self._raw_msg.info.resolution, 1e-6))))
        kernel_len = max(5, min(41, radius_px * 2 + 1))
        if kernel_len % 2 == 0:
            kernel_len += 1
        kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_len, 1))
        kernel_v = cv2.getStructuringElement(cv2.MORPH_RECT, (1, kernel_len))
        closed_h = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel_h)
        closed_v = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel_v)
        healed_mask = cv2.bitwise_or(closed_h, closed_v)
        fill_candidates = np.logical_and(healed_mask > 0, patch_mask > 0)
        self._overlay[fill_candidates] = 0

    def _fill_polygon(self, world_points, value):
        if self._overlay is None or not world_points:
            return
        polygon = np.array([[self._world_to_grid(pt["x"], pt["y"])[1], self._world_to_grid(pt["x"], pt["y"])[0]] for pt in world_points], dtype=np.int32)
        cv2.fillPoly(self._overlay, [polygon], int(value))

    def _world_to_grid(self, x, y):
        info = self._raw_msg.info
        col = int(round((x - info.origin.position.x) / max(info.resolution, 1e-6)))
        row = int(round((y - info.origin.position.y) / max(info.resolution, 1e-6)))
        info = self._raw_msg.info
        col = max(0, min(info.width - 1, col))
        row = max(0, min(info.height - 1, row))
        return row, col

    def _occupancy_to_bgr(self, grid):
        image = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
        image[grid == 0] = (245, 245, 245)
        image[grid == 100] = (45, 45, 45)
        image[grid < 0] = (180, 180, 180)
        return cv2.flip(image, 0)

    def _draw_regions(self, image, origin_x=None, origin_y=None, resolution=None, width=None, height=None, alignment_yaw=None):
        work_regions = sorted(self._work_regions.values(), key=lambda item: int(item.get("order_index", 0)))
        obstacle_regions = sorted(self._obstacle_regions.values(), key=lambda item: int(item.get("order_index", 0)))
        erase_regions = sorted(self._erase_regions.values(), key=lambda item: int(item.get("order_index", 0)))
        for region in work_regions:
            if not bool(region.get("enabled", True)):
                continue
            self._draw_polygon(image, region.get("points", []), (0, 200, 0), self._region_label(region, "work"), origin_x, origin_y, resolution, width, height, alignment_yaw)
        for region in obstacle_regions:
            if not bool(region.get("enabled", True)):
                continue
            self._draw_polygon(image, region.get("points", []), (0, 0, 220), self._region_label(region, "obstacle"), origin_x, origin_y, resolution, width, height, alignment_yaw)
        for region in erase_regions:
            if not bool(region.get("enabled", True)):
                continue
            self._draw_polygon(image, region.get("points", []), (0, 165, 255), self._region_label(region, "erase"), origin_x, origin_y, resolution, width, height, alignment_yaw)
        if self._crop_region is not None and bool(self._crop_region.get("enabled", True)):
            self._draw_polygon(
                image,
                self._crop_region.get("points", []),
                (255, 220, 0),
                self._region_label(self._crop_region, "crop"),
                origin_x,
                origin_y,
                resolution,
                width,
                height,
                alignment_yaw,
            )

    def _region_label(self, region, fallback_name):
        if not self._draw_region_label_on_preview:
            return ""
        if not isinstance(region, dict):
            return str(fallback_name or "")
        name = str(region.get("name", "") or "").strip()
        region_id = str(region.get("region_id", "") or "").strip()
        if self._draw_region_id_on_preview and region_id:
            return "{}({})".format(name, region_id) if name else region_id
        return name or region_id or str(fallback_name or "")

    def _draw_polygon(self, image, points, color, name, origin_x=None, origin_y=None, resolution=None, width=None, height=None, alignment_yaw=None):
        if not points:
            return
        polygon = np.array(
            [
                [
                    self._world_to_grid_for(pt["x"], pt["y"], origin_x, origin_y, resolution, width, height, alignment_yaw)[1],
                    self._world_to_grid_for(pt["x"], pt["y"], origin_x, origin_y, resolution, width, height, alignment_yaw)[0],
                ]
                for pt in points
            ],
            dtype=np.int32,
        )
        polygon[:, 1] = image.shape[0] - 1 - polygon[:, 1]
        cv2.polylines(image, [polygon], isClosed=True, color=color, thickness=1)
        text = str(name or "").strip()
        if text:
            x, y = polygon[0]
            cv2.putText(image, text[:12], (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)

    def _draw_pose(self, image, pose, origin_x=None, origin_y=None, resolution=None, width=None, height=None, alignment_yaw=None):
        if not pose or self._raw_msg is None:
            return
        row, col = self._world_to_grid_for(pose["x"], pose["y"], origin_x, origin_y, resolution, width, height, alignment_yaw)
        image_row = image.shape[0] - 1 - row
        display_yaw = math.radians(float(pose.get("heading_deg", 0.0)))
        align_yaw = self._sanitize_alignment_yaw(alignment_yaw)
        if align_yaw is not None:
            display_yaw += align_yaw
        arrow_len = max(10, min(28, int(round(min(image.shape[0], image.shape[1]) * 0.04))))
        end_col = int(round(float(col) + math.cos(display_yaw) * arrow_len))
        end_row = int(round(float(row) + math.sin(display_yaw) * arrow_len))
        end_col = max(0, min(image.shape[1] - 1, end_col))
        end_image_row = max(0, min(image.shape[0] - 1, image.shape[0] - 1 - end_row))
        cv2.arrowedLine(image, (int(col), int(image_row)), (int(end_col), int(end_image_row)), (255, 0, 0), 2, cv2.LINE_AA, tipLength=0.35)
        cv2.circle(image, (int(col), int(image_row)), 3, (255, 0, 0), thickness=-1)

    def _resize_with_aspect(self, image, max_edge):
        max_edge = max(64, int(max_edge))
        height, width = image.shape[:2]
        scale = min(float(max_edge) / max(width, height), 1.0)
        new_size = (max(1, int(round(width * scale))), max(1, int(round(height * scale))))
        if new_size == (width, height):
            return image
        if self._preview_rga_enabled:
            resized = self._resize_with_aspect_rga(image, new_size)
            if resized is not None:
                return resized
        return cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)

    def _resize_with_aspect_rga(self, image, new_size):
        if not self._preview_rga_available:
            self._warn_preview_rga_fallback("RGA device/backend unavailable")
            return None
        self._warn_preview_rga_fallback("Python librga resize backend is not wired")
        return None

    def _warn_preview_rga_fallback(self, reason):
        if self._preview_rga_fallback_warned:
            return
        self._preview_rga_fallback_warned = True
        if rospy is not None:
            rospy.logwarn(
                "preview_rga_enabled=true but %s; fallback to OpenCV resize/crop/rotate",
                reason,
            )

    def _serialize_regions(self, alignment_yaw=None):
        work_regions = sorted(self._work_regions.values(), key=lambda item: int(item.get("order_index", 0)))
        obstacle_regions = sorted(self._obstacle_regions.values(), key=lambda item: int(item.get("order_index", 0)))
        erase_regions = sorted(self._erase_regions.values(), key=lambda item: int(item.get("order_index", 0)))
        display_yaw = self._sanitize_alignment_yaw(alignment_yaw)
        if display_yaw is not None:
            work_regions = [self._region_to_display_region(region, display_yaw) for region in work_regions]
            obstacle_regions = [self._region_to_display_region(region, display_yaw) for region in obstacle_regions]
            erase_regions = [self._region_to_display_region(region, display_yaw) for region in erase_regions]
            crop_region = self._region_to_display_region(self._crop_region, display_yaw) if self._crop_region is not None else None
        else:
            crop_region = self._crop_region
        return {
            "work_regions": work_regions,
            "obstacle_regions": obstacle_regions,
            "erase_regions": erase_regions,
            "crop_region": crop_region,
        }

    def _build_overlay_json(
        self,
        preview_width,
        preview_height,
        raw_width,
        raw_height,
        origin_x=None,
        origin_y=None,
        resolution=None,
        frame_id=None,
        alignment_yaw=None,
        robot_pose=None,
    ):
        overlay_mask = np.where(self._overlay == UNKNOWN_MASK_VALUE, 0, 255).astype(np.uint8)
        raw_info = self._raw_msg.info if self._raw_msg is not None else None
        display_yaw = self._sanitize_alignment_yaw(alignment_yaw)
        if display_yaw is not None and raw_info is not None:
            overlay_mask, _, _ = self._rotate_grid_to_aligned_frame(
                overlay_mask,
                float(raw_info.origin.position.x),
                float(raw_info.origin.position.y),
                float(raw_info.resolution),
                display_yaw,
                fill_value=0,
            )
        overlay_mask = cv2.flip(overlay_mask, 0)
        overlay_mask = self._resize_with_aspect(overlay_mask, max(preview_width, preview_height))
        ok, buffer = cv2.imencode(".png", overlay_mask)
        mask_b64 = base64.b64encode(buffer.tobytes()).decode("ascii") if ok else ""
        scale_x = float(preview_width) / float(max(1, raw_width))
        scale_y = float(preview_height) / float(max(1, raw_height))
        payload = {
            "map_version": self._map_version,
            "updated_at": int(time.time()),
            "preview_width_px": int(preview_width),
            "preview_height_px": int(preview_height),
            "raw_width": int(raw_width),
            "raw_height": int(raw_height),
            "preview_scale_x": scale_x,
            "preview_scale_y": scale_y,
            "frame_id": str(frame_id or ""),
            "origin_x": float(origin_x or 0.0),
            "origin_y": float(origin_y or 0.0),
            "resolution": float(resolution or 0.0),
            "alignment_yaw": float(display_yaw or 0.0),
            "robot_pose": robot_pose or {},
            "regions": self._serialize_regions(display_yaw),
            "overlay_mask_png_base64": mask_b64,
            "last_edit_message": self._last_edit_message,
        }
        return json.dumps(payload, ensure_ascii=False)

    @staticmethod
    def _sanitize_alignment_yaw(alignment_yaw):
        if alignment_yaw is None:
            return None
        try:
            yaw = float(alignment_yaw)
        except Exception:
            return None
        if not math.isfinite(yaw) or abs(yaw) <= 1e-6:
            return None
        return yaw

    @staticmethod
    def _transform_map_point_to_aligned(x, y, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return cos_yaw * float(x) - sin_yaw * float(y), sin_yaw * float(x) + cos_yaw * float(y)

    def _pose_to_display_pose(self, pose, alignment_yaw=None):
        if not isinstance(pose, dict):
            return {}
        try:
            x = float(pose.get("x", 0.0))
            y = float(pose.get("y", 0.0))
            heading_deg = float(pose.get("heading_deg", 0.0))
        except Exception:
            return {}
        display_yaw = self._sanitize_alignment_yaw(alignment_yaw)
        if display_yaw is not None:
            x, y = self._transform_map_point_to_aligned(x, y, display_yaw)
            heading_deg += math.degrees(display_yaw)
        return {
            "x": x,
            "y": y,
            "heading_deg": heading_deg,
        }

    def _region_to_display_region(self, region, alignment_yaw):
        if not isinstance(region, dict):
            return region
        display = dict(region)
        display_points = []
        for point in region.get("points", []) or []:
            point_x, point_y = self._transform_map_point_to_aligned(
                point.get("x", 0.0),
                point.get("y", 0.0),
                alignment_yaw,
            )
            display_points.append({"x": point_x, "y": point_y})
        display["points"] = display_points
        for key in ("start_pose", "end_pose"):
            pose = self._pose_to_display_pose(region.get(key, {}), alignment_yaw)
            if pose:
                display[key] = pose
        return display

    def _world_to_grid_for(self, x, y, origin_x=None, origin_y=None, resolution=None, width=None, height=None, alignment_yaw=None):
        if origin_x is None or origin_y is None or resolution is None or width is None or height is None:
            return self._world_to_grid(x, y)
        display_yaw = self._sanitize_alignment_yaw(alignment_yaw)
        if display_yaw is not None:
            x, y = self._transform_map_point_to_aligned(x, y, display_yaw)
        resolution = max(float(resolution), 1e-6)
        col = int(round((float(x) - float(origin_x)) / resolution))
        row = int(round((float(y) - float(origin_y)) / resolution))
        col = max(0, min(int(width) - 1, col))
        row = max(0, min(int(height) - 1, row))
        return row, col

    @staticmethod
    def _rotate_grid_to_aligned_frame(grid, origin_x, origin_y, resolution, yaw, fill_value=-1):
        height, width = grid.shape[:2]
        if height <= 0 or width <= 0:
            return grid, origin_x, origin_y

        resolution = max(float(resolution), 1e-12)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rot = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]], dtype=np.float64)

        x0 = float(origin_x)
        y0 = float(origin_y)
        x1 = x0 + float(width) * resolution
        y1 = y0 + float(height) * resolution
        corners = np.array([[x0, y0], [x1, y0], [x0, y1], [x1, y1]], dtype=np.float64)
        rotated = corners @ rot.T

        min_x = float(rotated[:, 0].min())
        max_x = float(rotated[:, 0].max())
        min_y = float(rotated[:, 1].min())
        max_y = float(rotated[:, 1].max())
        out_width = max(1, int(math.ceil((max_x - min_x) / resolution)))
        out_height = max(1, int(math.ceil((max_y - min_y) / resolution)))

        cols = np.arange(out_width, dtype=np.float64)
        rows = np.arange(out_height, dtype=np.float64)
        x_aligned = min_x + (cols + 0.5) * resolution
        y_aligned = min_y + (rows + 0.5) * resolution
        xx, yy = np.meshgrid(x_aligned, y_aligned)

        x_map = cos_yaw * xx + sin_yaw * yy
        y_map = -sin_yaw * xx + cos_yaw * yy
        src_col = np.floor((x_map - x0) / resolution).astype(np.int64)
        src_row = np.floor((y_map - y0) / resolution).astype(np.int64)
        valid = (
            (src_col >= 0)
            & (src_col < int(width))
            & (src_row >= 0)
            & (src_row < int(height))
        )

        out_grid = np.full((out_height, out_width), fill_value, dtype=grid.dtype)
        if int(np.count_nonzero(valid)) > 0:
            out_grid[valid] = grid[src_row[valid], src_col[valid]]
        return out_grid, float(min_x), float(min_y)

    def save_local_state(self, state_dir):
        with self._lock:
            os.makedirs(state_dir, exist_ok=True)
            meta = {
                "map_version": self._map_version,
                "last_edit_message": self._last_edit_message,
                "regions": self._serialize_regions(),
            }
            if self._overlay is not None:
                meta["overlay_shape"] = [int(self._overlay.shape[0]), int(self._overlay.shape[1])]
                np.save(os.path.join(state_dir, "overlay.npy"), self._overlay.astype(np.int16))
            tmp_meta = os.path.join(state_dir, "map_overlay_state.json.tmp")
            final_meta = os.path.join(state_dir, "map_overlay_state.json")
            with open(tmp_meta, "w", encoding="utf-8") as handle:
                json.dump(meta, handle, ensure_ascii=False, indent=2)
            os.replace(tmp_meta, final_meta)

    def load_local_state(self, state_dir):
        with self._lock:
            meta_path = os.path.join(state_dir, "map_overlay_state.json")
            if not os.path.exists(meta_path):
                return False
            with open(meta_path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
            regions = data.get("regions", {})
            self._work_regions = {}
            self._obstacle_regions = {}
            self._erase_regions = {}
            self._crop_region = None
            for index, item in enumerate(regions.get("work_regions", [])):
                region_id = (item.get("region_id") or "").strip() or "work_region_{}".format(index + 1)
                self._work_regions[region_id] = {
                    "name": item.get("name", "work_region_{}".format(index + 1)),
                    "points": item.get("points", []),
                    "region_id": region_id,
                    "order_index": int(item.get("order_index", item.get("priority", (index + 1) * 10))),
                    "enabled": bool(item.get("enabled", True)),
                    "color_argb": int(item.get("color_argb", 0)),
                    "closed": bool(item.get("closed", True)),
                    "region_type": int(item.get("region_type", 1)),
                    "global_direction": str(item.get("global_direction", "x") or "x").strip().lower(),
                    "start_pose": dict(item.get("start_pose", {}) or {}),
                    "end_pose": dict(item.get("end_pose", {}) or {}),
                }
            for index, item in enumerate(regions.get("obstacle_regions", [])):
                if int(item.get("region_type", 2)) != 2:
                    continue
                region_id = (item.get("region_id") or "").strip() or "obstacle_region_{}".format(index + 1)
                self._obstacle_regions[region_id] = {
                    "name": item.get("name", "obstacle_region_{}".format(index + 1)),
                    "points": item.get("points", []),
                    "region_id": region_id,
                    "order_index": int(item.get("order_index", item.get("priority", (index + 1) * 10))),
                    "enabled": bool(item.get("enabled", True)),
                    "color_argb": int(item.get("color_argb", 0)),
                    "closed": bool(item.get("closed", True)),
                    "region_type": 2,
                }
            for index, item in enumerate(regions.get("erase_regions", [])):
                if int(item.get("region_type", 3)) != 3:
                    continue
                region_id = (item.get("region_id") or "").strip() or "erase_region_{}".format(index + 1)
                self._erase_regions[region_id] = {
                    "name": item.get("name", "erase_region_{}".format(index + 1)),
                    "points": item.get("points", []),
                    "region_id": region_id,
                    "order_index": int(item.get("order_index", item.get("priority", (index + 1) * 10))),
                    "enabled": bool(item.get("enabled", True)),
                    "color_argb": int(item.get("color_argb", 0)),
                    "closed": bool(item.get("closed", True)),
                    "region_type": 3,
                }
            crop = regions.get("crop_region")
            if isinstance(crop, dict):
                points = crop.get("points", []) or []
                self._crop_region = {
                    "name": crop.get("name", "crop_region"),
                    "points": self._normalize_rect_points(points),
                    "region_id": "crop_region_1",
                    "order_index": int(crop.get("order_index", 10)),
                    "enabled": bool(crop.get("enabled", True)),
                    "color_argb": int(crop.get("color_argb", 0)),
                    "closed": bool(crop.get("closed", True)),
                    "region_type": 4,
                }
            self._map_version = int(data.get("map_version", self._map_version))
            self._last_edit_message = data.get("last_edit_message", self._last_edit_message)

            overlay_path = os.path.join(state_dir, "overlay.npy")
            if os.path.exists(overlay_path):
                loaded = np.load(overlay_path).astype(np.int16)
                if self._raw_msg is not None:
                    shape = (self._raw_msg.info.height, self._raw_msg.info.width)
                    if loaded.shape == shape:
                        self._overlay = loaded
                    else:
                        self._pending_overlay = loaded
                else:
                    self._pending_overlay = loaded
            return True

    def reset_local_state(self):
        with self._lock:
            self._work_regions = {}
            self._obstacle_regions = {}
            self._erase_regions = {}
            self._crop_region = None
            self._last_edit_message = ""

    def reset_overlay_regions(self):
        """仅清空区域覆盖层（工作区/障碍区/擦除区/裁剪区），不影响原始底图缓存。"""
        with self._lock:
            self._work_regions = {}
            self._obstacle_regions = {}
            self._erase_regions = {}
            self._crop_region = None
            self._last_edit_message = "overlay_regions_cleared"
            self._map_version = 0
            if self._raw_msg is not None:
                shape = (self._raw_msg.info.height, self._raw_msg.info.width)
                self._overlay = np.full(shape, UNKNOWN_MASK_VALUE, dtype=np.int16)
                self._pending_overlay = None
            else:
                self._overlay = None
                self._pending_overlay = None
