import contextlib
import importlib.util
import io
import math
import os
import time
from pathlib import Path as FsPath

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath

from grinder_scheduler.models import (
    PlannerPath,
    normalize_planning_direction,
    planning_direction_axis,
)


class PlannerAdapter:
    def __init__(self, planner_script_path):
        self._planner_script_path = FsPath(planner_script_path)
        self._planner_module = None
        self._hybrid_module = None
        self._planner_import_error = None
        self._hybrid_import_error = None
        try:
            self._planner_module = self._load_planner_module(self._planner_script_path)
        except Exception as exc:
            self._planner_import_error = exc
            rospy.logwarn("Planner module import failed, using fallback planner: %s", exc)
        try:
            hybrid_candidates = [
                self._planner_script_path.parent / "hybrid_planner_adaptive.py",
                self._planner_script_path.parent.parent / "hybrid_planner_adaptive.py",
            ]
            hybrid_script = next((path for path in hybrid_candidates if path.exists()), None)
            if hybrid_script is not None:
                self._hybrid_module = self._load_planner_module(hybrid_script)
        except Exception as exc:
            self._hybrid_import_error = exc
            rospy.logwarn("Hybrid connector module import failed, falling back to legacy connector: %s", exc)
        self._path_version = 0
        self._last_runtime_backend = ""
        if self._planner_module is not None:
            cpp_module = getattr(self._planner_module, "_mst27_cpp", None)
            cpp_disabled = os.environ.get("MST27_DISABLE_CPP") == "1"
            if cpp_module is not None and not cpp_disabled:
                rospy.loginfo(
                    "MST27 planner backend ready: backend=cpp version=%s module=%s",
                    str(getattr(cpp_module, "__version__", "unknown")),
                    str(getattr(cpp_module, "__file__", "<unknown>")),
                )
            else:
                rospy.logwarn(
                    "MST27 planner backend ready: backend=python_fallback cpp_loaded=%s cpp_disabled=%s import_error=%s",
                    str(cpp_module is not None).lower(),
                    str(cpp_disabled).lower(),
                    str(getattr(self._planner_module, "_mst27_cpp_import_error", "") or "<none>"),
                )

    def _load_planner_module(self, script_path):
        spec = importlib.util.spec_from_file_location("grinder_mst25", str(script_path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def _yaw_to_quaternion(self, yaw):
        half = 0.5 * float(yaw)
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(half),
            "w": math.cos(half),
        }

    def _extract_orientation(self, point):
        ori = getattr(point, "orientation", None)
        if isinstance(ori, dict):
            try:
                return {
                    "x": float(ori.get("x", 0.0)),
                    "y": float(ori.get("y", 0.0)),
                    "z": float(ori.get("z", 0.0)),
                    "w": float(ori.get("w", 1.0)),
                }
            except Exception:
                return None
        if ori is None:
            return None
        try:
            return {
                "x": float(getattr(ori, "x", 0.0)),
                "y": float(getattr(ori, "y", 0.0)),
                "z": float(getattr(ori, "z", 0.0)),
                "w": float(getattr(ori, "w", 1.0)),
            }
        except Exception:
            return None

    def plan(self, task_id, composed_grid, map_info, task_config):
        t0_total = time.perf_counter()
        if self._planner_module is None:
            if self._planner_import_error is not None:
                rospy.logwarn_throttle(5.0, "Using fallback planner due to import error: %s", self._planner_import_error)
            return self._fallback_plan(task_id, composed_grid, map_info, task_config)

        module = self._planner_module
        resolution = map_info["resolution"]
        ratio = 1.0 / max(resolution, 1e-6)
        work_regions = task_config.work_regions
        if not work_regions:
            auto_regions = self._build_free_space_regions(composed_grid, map_info)
            if auto_regions:
                work_regions = auto_regions
                rospy.logwarn(
                    "No work region provided. Auto-derived %d region(s) from free space contours.",
                    len(auto_regions),
                )
            else:
                work_regions = [self._build_full_map_region(composed_grid.shape, resolution, map_info)]
                rospy.logwarn("No work region provided and free-space contour extraction failed. Fallback to full-map region.")
        grid_map = np.where(composed_grid != 0, 1, 0).astype(int)
        occupied_before_regions = int(np.count_nonzero(grid_map))
        self._apply_erase_regions_to_grid_map(
            grid_map,
            map_info,
            getattr(task_config, "erase_regions", []),
        )
        obstacle_cells = self._apply_obstacle_regions_to_grid_map(
            grid_map,
            map_info,
            getattr(task_config, "obstacle_regions", []),
        )
        # Crop is the final planning boundary and must not be reopened by an erase region.
        self._apply_crop_region_to_grid_map(grid_map, map_info, getattr(task_config, "crop_region", {}))
        rospy.loginfo(
            "Planning obstacle grid applied: regions=%d mask_cells=%d occupied_before=%d occupied_final=%d",
            len(list(getattr(task_config, "obstacle_regions", []) or [])),
            obstacle_cells,
            occupied_before_regions,
            int(np.count_nonzero(grid_map)),
        )
        t1_grid = time.perf_counter()

        robot_config = module.RobotConfig(
            width=max(0.1, float(getattr(task_config, "vehicle_width", 0.5))),
            length=max(0.1, float(getattr(task_config, "vehicle_length", 0.5))),
            path_spacing=max(task_config.default_path_spacing, 0.2),
            turning_radius=max(task_config.turn_radius, 0.1),
            overlap_ratio=max(0.0, min(task_config.overlap_ratio, 0.95)),
            inflation_radius=max(task_config.inflation_radius, 0.0),
        )
        planner = module.MultiStagePathPlanner(robot_config)
        inflation_pixels = int(max(0.0, float(robot_config.inflation_radius)) * ratio)
        inflated_for_connection = module.ObstacleProcessor.inflate_obstacles(grid_map, inflation_pixels)
        t2_inflate = time.perf_counter()
        path_points = []
        current_pose = getattr(task_config, "current_pose", {}) or {}
        robot_x = float(current_pose.get("x", 0.0)) if isinstance(current_pose, dict) else 0.0
        robot_y = float(current_pose.get("y", 0.0)) if isinstance(current_pose, dict) else 0.0
        request_start_pose = getattr(task_config, "start_pose", {}) or {}
        request_end_pose = getattr(task_config, "end_pose", {}) or {}
        has_task_info = bool(list(getattr(task_config, "selected_work_region_ids", []) or [])) or bool(
            dict(getattr(task_config, "region_repeat_config", {}) or {})
        )
        start_x = float(request_start_pose.get("x", 0.0)) if isinstance(request_start_pose, dict) else 0.0
        start_y = float(request_start_pose.get("y", 0.0)) if isinstance(request_start_pose, dict) else 0.0
        end_x = float(request_end_pose.get("x", 0.0)) if isinstance(request_end_pose, dict) else 0.0
        end_y = float(request_end_pose.get("y", 0.0)) if isinstance(request_end_pose, dict) else 0.0
        has_request_start = bool(request_start_pose)
        has_request_end = bool(request_end_pose)
        direction_cfg = normalize_planning_direction(
            getattr(task_config, "global_direction", "x")
        )
        planning_angle_deg = getattr(task_config, "planning_angle_deg", None)
        try:
            planning_angle_deg = float(planning_angle_deg)
            if not math.isfinite(planning_angle_deg):
                planning_angle_deg = None
        except (TypeError, ValueError):
            planning_angle_deg = None
        rospy.loginfo(
            "PlannerAdapter input start candidates: current_pose=(%.3f,%.3f) request_start=%s direction=%s planning_angle_deg=%s has_task_info=%s",
            robot_x,
            robot_y,
            (
                "({:.3f},{:.3f})".format(start_x, start_y)
                if has_request_start
                else "<empty>"
            ),
            direction_cfg,
            "{:.6f}".format(planning_angle_deg) if planning_angle_deg is not None else "<axis_only>",
            str(bool(has_task_info)).lower(),
        )
        planned_region_segments = []
        for index, region in enumerate(work_regions, start=1):
            region_id = str(region.get("region_id", "")).strip() if isinstance(region, dict) else ""
            region_tag = region_id if region_id else "region_{}".format(index)
            boundary = [self._world_to_point(module, point["x"], point["y"], map_info, ratio) for point in region["points"]]
            if len(boundary) < 3:
                continue
            region_start_pose = dict(region.get("start_pose", {}) or {}) if isinstance(region, dict) else {}
            region_end_pose = dict(region.get("end_pose", {}) or {}) if isinstance(region, dict) else {}
            start_point = None
            start_source = "default_boundary"
            if has_request_start and self._world_point_in_region(start_x, start_y, region):
                start_point = self._world_to_point(module, start_x, start_y, map_info, ratio)
                start_source = "request_start"
                rospy.loginfo(
                    "Region planning start from request start_pose: region_idx=%d region_id=%s start=(%.3f,%.3f) start_grid=(%d,%d)",
                    index,
                    region_id or "<empty>",
                    start_x,
                    start_y,
                    int(start_point.row),
                    int(start_point.col),
                )
            elif region_start_pose and self._world_point_in_region(float(region_start_pose.get("x", 0.0)), float(region_start_pose.get("y", 0.0)), region):
                sx = float(region_start_pose.get("x", 0.0))
                sy = float(region_start_pose.get("y", 0.0))
                start_point = self._world_to_point(module, sx, sy, map_info, ratio)
                start_source = "region_start"
                rospy.loginfo(
                    "Region planning start from region start_pose: region_idx=%d region_id=%s start=(%.3f,%.3f) start_grid=(%d,%d)",
                    index,
                    region_id or "<empty>",
                    sx,
                    sy,
                    int(start_point.row),
                    int(start_point.col),
                )
            elif self._world_point_in_region(robot_x, robot_y, region):
                start_point = self._world_to_point(module, robot_x, robot_y, map_info, ratio)
                start_source = "current_pose"
                rospy.loginfo(
                    "Region planning start from robot pose: region_idx=%d region_id=%s robot=(%.3f,%.3f) start_grid=(%d,%d)",
                    index,
                    region_id or "<empty>",
                    robot_x,
                    robot_y,
                    int(start_point.row),
                    int(start_point.col),
                )
            end_point = boundary[-1]
            if has_request_end and self._world_point_in_region(end_x, end_y, region):
                end_point = self._world_to_point(module, end_x, end_y, map_info, ratio)
                rospy.loginfo(
                    "Region planning end from request end_pose: region_idx=%d region_id=%s end=(%.3f,%.3f) end_grid=(%d,%d)",
                    index,
                    region_id or "<empty>",
                    end_x,
                    end_y,
                    int(end_point.row),
                    int(end_point.col),
                )
            elif region_end_pose and self._world_point_in_region(float(region_end_pose.get("x", 0.0)), float(region_end_pose.get("y", 0.0)), region):
                ex = float(region_end_pose.get("x", 0.0))
                ey = float(region_end_pose.get("y", 0.0))
                end_point = self._world_to_point(module, ex, ey, map_info, ratio)
                rospy.loginfo(
                    "Region planning end from region end_pose: region_idx=%d region_id=%s end=(%.3f,%.3f) end_grid=(%d,%d)",
                    index,
                    region_id or "<empty>",
                    ex,
                    ey,
                    int(end_point.row),
                    int(end_point.col),
                )
            region_direction = normalize_planning_direction(
                region.get("global_direction", "") if isinstance(region, dict) else "",
                direction_cfg,
            )
            region_angle_deg = planning_angle_deg
            if region_angle_deg is not None and planning_direction_axis(region_direction) == "y":
                region_angle_deg += 90.0
            single_stage = module.StageConfig(
                stage_id=1,
                boundary_polygon=boundary,
                start_point=start_point,
                end_point=end_point,
                direction=region_direction,
                path_spacing=max(1.0, task_config.default_path_spacing * ratio),
                obstacle_polygons=None,
                direction_angle=region_angle_deg,
            )
            rospy.loginfo(
                "Region planning direction: region_idx=%d region_id=%s direction=%s direction_angle_deg=%s",
                index,
                region_id or "<empty>",
                region_direction,
                "{:.6f}".format(region_angle_deg) if region_angle_deg is not None else "<axis_only>",
            )
            if start_point is not None:
                rospy.loginfo(
                    "Region planning effective start: region_idx=%d region_id=%s source=%s start_grid=(%d,%d) start_world=(%.3f,%.3f)",
                    index,
                    region_id or "<empty>",
                    start_source,
                    int(start_point.row),
                    int(start_point.col),
                    float(start_point.col) * resolution + map_info["origin_x"],
                    float(start_point.row) * resolution + map_info["origin_y"],
                )
            else:
                rospy.loginfo(
                    "Region planning effective start: region_idx=%d region_id=%s source=%s start_grid=<auto_by_planner>",
                    index,
                    region_id or "<empty>",
                    start_source,
                )
            planner_console = io.StringIO()
            with contextlib.redirect_stdout(planner_console):
                t_region_plan_start = time.perf_counter()
                region_points = planner.plan(
                    grid_map.copy(),
                    boundary,
                    [single_stage],
                    ratio,
                    global_obstacle_polygons=None,
                )
                t_region_plan_end = time.perf_counter()
            planner_output = planner_console.getvalue()
            cpp_failed = "[C++后端] 调用失败" in planner_output
            runtime_backend = "cpp" if "[C++后端] 使用C++规划内核" in planner_output else "python_fallback"
            if runtime_backend != self._last_runtime_backend:
                log_method = rospy.logwarn if cpp_failed else rospy.loginfo
                log_method(
                    "MST27 planner runtime backend: backend=%s cpp_failed=%s",
                    runtime_backend,
                    str(cpp_failed).lower(),
                )
                self._last_runtime_backend = runtime_backend
            if cpp_failed:
                failure_line = next(
                    (line.strip() for line in planner_output.splitlines() if "[C++后端] 调用失败" in line),
                    "[C++后端] 调用失败",
                )
                rospy.logwarn("MST27 C++ backend fallback reason: %s", failure_line)
            # Keep each region's coverage inside its boundary. Build the contour
            # once: rebuilding the numpy contour for every path point can stall a
            # large 0x0505 request long enough that no 0x0506 chunks are returned.
            raw_region_point_count = len(region_points)
            region_polygon = np.array(
                [
                    [
                        float((p["x"] - map_info["origin_x"]) * ratio),
                        float((p["y"] - map_info["origin_y"]) * ratio),
                    ]
                    for p in list(region.get("points", []) or [])
                ],
                dtype=np.float32,
            )
            if len(region_polygon) >= 3:
                region_points = [
                    point
                    for point in region_points
                    if cv2.pointPolygonTest(
                        region_polygon,
                        (float(point.col), float(point.row)),
                        False,
                    )
                    >= 0
                ]
            else:
                region_points = []
            t_region_filter = time.perf_counter()
            rospy.loginfo(
                "Single-region path filtered: region_idx=%d region_id=%s raw_points=%d kept_points=%d filter_ms=%.1f",
                index,
                region_id or "<empty>",
                raw_region_point_count,
                len(region_points),
                (t_region_filter - t_region_plan_end) * 1000.0,
            )
            if not region_points:
                continue
            normalized_region_points = []
            for point in region_points:
                try:
                    point.path_type = region_tag
                    normalized_region_points.append(point)
                except Exception:
                    normalized_region_points.append(
                        module.PathPoint(
                            row=float(point.row),
                            col=float(point.col),
                            time=float(getattr(point, "time", 0.0)),
                            point_type=str(getattr(point, "point_type", "intermediate")),
                            path_type=region_tag,
                        )
                    )
            region_points = normalized_region_points
            planned_region_segments.append(
                {
                    "region_id": region_id,
                    "region_tag": region_tag,
                    "region_index": index,
                    "points": region_points,
                }
            )
            rospy.loginfo(
                "Single-region planning prepared: region_idx=%d region_id=%s points=%d algorithm=%.1fms filter=%.1fms normalize=%.1fms",
                index,
                str(region.get("region_id", "")).strip() or "<empty>",
                len(region_points),
                (t_region_plan_end - t_region_plan_start) * 1000.0,
                (t_region_filter - t_region_plan_end) * 1000.0,
                (time.perf_counter() - t_region_filter) * 1000.0,
            )

        t3_regions = time.perf_counter()
        if has_task_info:
            selected_ids = [str(x).strip() for x in list(getattr(task_config, "selected_work_region_ids", []) or []) if str(x).strip()]
            repeat_cfg = dict(getattr(task_config, "region_repeat_config", {}) or {})
            segment_by_id = {str(item.get("region_id", "")).strip(): item for item in planned_region_segments if str(item.get("region_id", "")).strip()}
            ordered_segments = []
            for region_id in selected_ids:
                segment = segment_by_id.get(region_id)
                if segment is None:
                    rospy.logwarn("Selected region not found in prepared segments: region_id=%s", region_id)
                    continue
                ordered_segments.append(segment)
            # With explicit selected_work_region_ids, execute only those regions in order.
            # Fallback to all prepared segments only when selection list is empty.
            if not selected_ids:
                ordered_segments = list(planned_region_segments)
            elif (not ordered_segments) and planned_region_segments:
                rospy.logwarn(
                    "No selected regions matched prepared segments; fallback to all prepared segments: selected=%s prepared=%d",
                    ",".join(selected_ids),
                    len(planned_region_segments),
                )
                ordered_segments = list(planned_region_segments)
            rospy.loginfo(
                "Compose global path by task order: selected_regions=%s prepared_regions=%d",
                ",".join(selected_ids) if selected_ids else "<empty>",
                len(ordered_segments),
            )
            last_end = None
            for seg_idx, segment in enumerate(ordered_segments, start=1):
                base_points = list(segment.get("points", []) or [])
                if not base_points:
                    continue
                region_id = str(segment.get("region_id", "")).strip() or str(segment.get("region_tag", "")).strip() or "region_{}".format(seg_idx)
                repeat = 1
                try:
                    repeat = int(repeat_cfg.get(region_id, 1))
                except Exception:
                    repeat = 1
                repeat = max(1, repeat)
                for lap in range(1, repeat + 1):
                    lap_points = list(base_points if (lap % 2 == 1) else list(reversed(base_points)))
                    for p in lap_points:
                        p.path_type = "{}__lap_{}".format(region_id, lap)
                    conn_goal = module.Point(
                        row=int(round(float(lap_points[0].row))),
                        col=int(round(float(lap_points[0].col))),
                    )
                    conn_start = None
                    if last_end is not None:
                        conn_start = module.Point(
                            row=int(round(float(last_end.row))),
                            col=int(round(float(last_end.col))),
                        )
                    elif seg_idx == 1 and lap == 1:
                        if has_request_start:
                            robot_start = self._world_to_point(module, start_x, start_y, map_info, ratio)
                        else:
                            robot_start = self._world_to_point(module, robot_x, robot_y, map_info, ratio)
                        conn_start = module.Point(
                            row=int(round(float(robot_start.row))),
                            col=int(round(float(robot_start.col))),
                        )
                    if conn_start is not None:
                        raw_conn_start = module.Point(row=int(conn_start.row), col=int(conn_start.col))
                        raw_conn_goal = module.Point(row=int(conn_goal.row), col=int(conn_goal.col))
                        conn_start = self._snap_point_to_free_cell(module, inflated_for_connection, conn_start)
                        conn_goal = self._snap_point_to_free_cell(module, inflated_for_connection, conn_goal)
                        if int(conn_start.row) != int(raw_conn_start.row) or int(conn_start.col) != int(raw_conn_start.col):
                            rospy.loginfo(
                                "Region connector start snapped: region_id=%s lap=%d from=(%d,%d) to=(%d,%d)",
                                region_id,
                                lap,
                                int(raw_conn_start.row),
                                int(raw_conn_start.col),
                                int(conn_start.row),
                                int(conn_start.col),
                            )
                        if int(conn_goal.row) != int(raw_conn_goal.row) or int(conn_goal.col) != int(raw_conn_goal.col):
                            rospy.loginfo(
                                "Region connector goal snapped: region_id=%s lap=%d from=(%d,%d) to=(%d,%d)",
                                region_id,
                                lap,
                                int(raw_conn_goal.row),
                                int(raw_conn_goal.col),
                                int(conn_goal.row),
                                int(conn_goal.col),
                            )
                        # Skip zero-length / near-zero connector to avoid drawing
                        # fake start-end links between adjacent laps in same region.
                        if abs(int(conn_start.row) - int(conn_goal.row)) <= 1 and abs(int(conn_start.col) - int(conn_goal.col)) <= 1:
                            rospy.loginfo(
                                "Region connector skipped (near same point): region_id=%s lap=%d from=(%d,%d) to=(%d,%d)",
                                region_id,
                                lap,
                                conn_start.row,
                                conn_start.col,
                                conn_goal.row,
                                conn_goal.col,
                            )
                            path_points.extend(lap_points)
                            last_end = lap_points[-1]
                            rospy.loginfo(
                                "Single-region path merged: region_id=%s lap=%d points=%d cumulative=%d",
                                region_id,
                                lap,
                                len(lap_points),
                                len(path_points),
                            )
                            continue
                        connector = self._plan_region_connector_hybrid(
                            module=module,
                            map_info=map_info,
                            task_config=task_config,
                            conn_start=conn_start,
                            conn_goal=conn_goal,
                        )
                        if not connector:
                            try:
                                connector = planner.planner_core.plan_connection_path(inflated_for_connection, conn_start, conn_goal)
                            except Exception as exc:
                                rospy.logwarn(
                                    "Region connector planning failed: from=(%s,%s) to=(%s,%s), err=%s",
                                    conn_start.row,
                                    conn_start.col,
                                    conn_goal.row,
                                    conn_goal.col,
                                    exc,
                                )
                                connector = []
                        if not connector:
                            try:
                                raw_start = self._snap_point_to_free_cell(module, grid_map, raw_conn_start)
                                raw_goal = self._snap_point_to_free_cell(module, grid_map, raw_conn_goal)
                                connector = planner.planner_core.plan_connection_path(grid_map, raw_start, raw_goal)
                                if connector:
                                    rospy.logwarn(
                                        "Region connector planned on non-inflated grid: region_id=%s lap=%d points=%d",
                                        region_id,
                                        lap,
                                        len(connector),
                                    )
                            except Exception as exc:
                                rospy.logwarn(
                                    "Region connector non-inflated fallback failed: from=(%s,%s) to=(%s,%s), err=%s",
                                    raw_conn_start.row,
                                    raw_conn_start.col,
                                    raw_conn_goal.row,
                                    raw_conn_goal.col,
                                    exc,
                                )
                                connector = []
                        if not connector:
                            connector = self._build_straight_connector(module, conn_start, conn_goal)
                            rospy.logwarn(
                                "Region connector straight fallback used: region_id=%s lap=%d from=(%d,%d) to=(%d,%d) points=%d",
                                region_id,
                                lap,
                                int(conn_start.row),
                                int(conn_start.col),
                                int(conn_goal.row),
                                int(conn_goal.col),
                                len(connector),
                            )
                        # Snapping on the inflated grid can move both connector ends
                        # away from the actual region path endpoints. Close the head
                        # gap from the previous region before handling the tail gap.
                        if connector:
                            connector_begin = module.Point(
                                row=int(round(float(connector[0].row))),
                                col=int(round(float(connector[0].col))),
                            )
                            raw_start = self._snap_point_to_free_cell(module, grid_map, raw_conn_start)
                            head_row_gap = abs(int(raw_start.row) - int(connector_begin.row))
                            head_col_gap = abs(int(raw_start.col) - int(connector_begin.col))
                            if head_row_gap > 1 or head_col_gap > 1:
                                connector_head = []
                                try:
                                    raw_head_goal = self._snap_point_to_free_cell(
                                        module,
                                        grid_map,
                                        connector_begin,
                                    )
                                    connector_head = planner.planner_core.plan_connection_path(
                                        grid_map,
                                        raw_start,
                                        raw_head_goal,
                                    )
                                except Exception as exc:
                                    rospy.logwarn(
                                        "Region connector head planning failed: region_id=%s lap=%d from=(%d,%d) to=(%d,%d), err=%s",
                                        region_id,
                                        lap,
                                        int(raw_start.row),
                                        int(raw_start.col),
                                        int(connector_begin.row),
                                        int(connector_begin.col),
                                        exc,
                                    )
                                if not connector_head:
                                    connector_head = self._build_straight_connector(
                                        module,
                                        raw_start,
                                        connector_begin,
                                    )
                                    rospy.logwarn(
                                        "Region connector head straight fallback used: region_id=%s lap=%d from=(%d,%d) to=(%d,%d) points=%d",
                                        region_id,
                                        lap,
                                        int(raw_start.row),
                                        int(raw_start.col),
                                        int(connector_begin.row),
                                        int(connector_begin.col),
                                        len(connector_head),
                                    )
                                merged_connector = []
                                for point in list(connector_head) + list(connector):
                                    if (
                                        merged_connector
                                        and int(round(float(merged_connector[-1].row))) == int(round(float(point.row)))
                                        and int(round(float(merged_connector[-1].col))) == int(round(float(point.col)))
                                    ):
                                        continue
                                    merged_connector.append(point)
                                connector = merged_connector
                                rospy.loginfo(
                                    "Region connector head completed: region_id=%s lap=%d gap_cells=(%d,%d) head_points=%d",
                                    region_id,
                                    lap,
                                    head_row_gap,
                                    head_col_gap,
                                    len(connector_head),
                                )

                        # Close the final short gap from the inflated-grid connector
                        # goal to the next region's actual first point on the raw map.
                        if connector:
                            connector_end = module.Point(
                                row=int(round(float(connector[-1].row))),
                                col=int(round(float(connector[-1].col))),
                            )
                            raw_goal = self._snap_point_to_free_cell(module, grid_map, raw_conn_goal)
                            row_gap = abs(int(connector_end.row) - int(raw_goal.row))
                            col_gap = abs(int(connector_end.col) - int(raw_goal.col))
                            if row_gap > 1 or col_gap > 1:
                                connector_tail = []
                                try:
                                    raw_start = self._snap_point_to_free_cell(module, grid_map, connector_end)
                                    connector_tail = planner.planner_core.plan_connection_path(
                                        grid_map,
                                        raw_start,
                                        raw_goal,
                                    )
                                except Exception as exc:
                                    rospy.logwarn(
                                        "Region connector tail planning failed: region_id=%s lap=%d from=(%d,%d) to=(%d,%d), err=%s",
                                        region_id,
                                        lap,
                                        int(connector_end.row),
                                        int(connector_end.col),
                                        int(raw_goal.row),
                                        int(raw_goal.col),
                                        exc,
                                    )
                                if not connector_tail:
                                    connector_tail = self._build_straight_connector(
                                        module,
                                        connector_end,
                                        raw_goal,
                                    )
                                    rospy.logwarn(
                                        "Region connector tail straight fallback used: region_id=%s lap=%d from=(%d,%d) to=(%d,%d) points=%d",
                                        region_id,
                                        lap,
                                        int(connector_end.row),
                                        int(connector_end.col),
                                        int(raw_goal.row),
                                        int(raw_goal.col),
                                        len(connector_tail),
                                    )
                                tail_added = 0
                                for point in connector_tail:
                                    if (
                                        connector
                                        and int(round(float(connector[-1].row))) == int(round(float(point.row)))
                                        and int(round(float(connector[-1].col))) == int(round(float(point.col)))
                                    ):
                                        continue
                                    connector.append(point)
                                    tail_added += 1
                                rospy.loginfo(
                                    "Region connector tail completed: region_id=%s lap=%d gap_cells=(%d,%d) added_points=%d",
                                    region_id,
                                    lap,
                                    row_gap,
                                    col_gap,
                                    tail_added,
                                )
                        if connector and len(connector) >= 2:
                            for point in connector:
                                path_points.append(
                                    module.PathPoint(
                                        row=float(point.row),
                                        col=float(point.col),
                                        time=float(getattr(point, "time", 0.0)),
                                        point_type="intermediate",
                                        path_type="connection",
                                    )
                                )
                        rospy.loginfo(
                            "Region connector: region_id=%s lap=%d from=(%d,%d) to=(%d,%d) connector_points=%d",
                            region_id,
                            lap,
                            conn_start.row,
                            conn_start.col,
                            conn_goal.row,
                            conn_goal.col,
                            len(connector),
                        )
                    path_points.extend(lap_points)
                    last_end = lap_points[-1]
                    rospy.loginfo(
                        "Single-region path merged: region_id=%s lap=%d points=%d cumulative=%d",
                        region_id,
                        lap,
                        len(lap_points),
                        len(path_points),
                    )
        else:
            for segment in planned_region_segments:
                path_points.extend(list(segment.get("points", []) or []))
            rospy.loginfo(
                "Compose global path without task info: regions=%d points=%d",
                len(planned_region_segments),
                len(path_points),
            )

        t4_compose = time.perf_counter()
        self._path_version += 1
        ros_path = NavPath()
        ros_path.header.frame_id = map_info["frame_id"] or "map"
        path_records = []
        length_m = 0.0
        prev = None
        cached_xy = []
        for point in path_points:
            cached_xy.append(
                (
                    float(point.col) * resolution + map_info["origin_x"],
                    float(point.row) * resolution + map_info["origin_y"],
                )
            )
        for idx, point in enumerate(path_points):
            x, y = cached_xy[idx]
            pose = PoseStamped()
            pose.header.frame_id = ros_path.header.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            quat = self._extract_orientation(point)
            if quat is None:
                if idx + 1 < len(cached_xy):
                    nx, ny = cached_xy[idx + 1]
                    yaw = math.atan2(ny - y, nx - x)
                elif idx > 0:
                    px, py = cached_xy[idx - 1]
                    yaw = math.atan2(y - py, x - px)
                else:
                    yaw = 0.0
                quat = self._yaw_to_quaternion(yaw)
            pose.pose.orientation.x = float(quat["x"])
            pose.pose.orientation.y = float(quat["y"])
            pose.pose.orientation.z = float(quat["z"])
            pose.pose.orientation.w = float(quat["w"])
            ros_path.poses.append(pose)
            point_type = getattr(point, "path_type", "") or "stage"
            path_records.append(
                {
                    "index": int(idx),
                    "row": float(point.row),
                    "col": float(point.col),
                    "x": x,
                    "y": y,
                    "path_type": point_type,
                    "point_type": str(getattr(point, "point_type", "intermediate")),
                    "timestamp": float(getattr(point, "time", 0.0)),
                    "orientation": quat,
                }
            )
            if prev is not None:
                length_m += math.hypot(x - prev[0], y - prev[1])
            prev = (x, y)
        t5_output = time.perf_counter()
        rospy.loginfo(
            "PlannerAdapter perf: total=%.1fms grid=%.1fms inflate=%.1fms regions=%.1fms compose=%.1fms output=%.1fms path_points=%d",
            (t5_output - t0_total) * 1000.0,
            (t1_grid - t0_total) * 1000.0,
            (t2_inflate - t1_grid) * 1000.0,
            (t3_regions - t2_inflate) * 1000.0,
            (t4_compose - t3_regions) * 1000.0,
            (t5_output - t4_compose) * 1000.0,
            len(path_points),
        )

        return PlannerPath(
            task_id=task_id,
            path_version=self._path_version,
            points=path_records,
            nav_path=ros_path,
            length_m=length_m,
        )

    def _extract_lap_index(self, region_id_text):
        text = str(region_id_text or "").strip()
        if "__lap_" not in text:
            return 1
        try:
            return max(1, int(text.rsplit("__lap_", 1)[1]))
        except Exception:
            return 1

    def _plan_region_connector_hybrid(self, module, map_info, task_config, conn_start, conn_goal):
        if self._hybrid_module is None:
            return []
        try:
            resolution = float(map_info["resolution"])
            origin_x = float(map_info["origin_x"])
            origin_y = float(map_info["origin_y"])
            sx = float(conn_start.col) * resolution + origin_x
            sy = float(conn_start.row) * resolution + origin_y
            gx = float(conn_goal.col) * resolution + origin_x
            gy = float(conn_goal.row) * resolution + origin_y
            # Use segment direction as yaw seed.
            yaw = math.atan2(gy - sy, gx - sx)
            q = self._yaw_to_quaternion(yaw)
            start = {"point": {"x": sx, "y": sy}, "orientation": q}
            goal = {"point": {"x": gx, "y": gy}, "orientation": q}

            obstacles = []
            for region in list(getattr(task_config, "obstacle_regions", []) or []):
                points = region.get("points", []) if isinstance(region, dict) else []
                if len(points) < 3:
                    continue
                poly_pts = []
                for p in points:
                    poly_pts.append((float(p["x"]), float(p["y"])))
                obstacles.append(self._hybrid_module.PolygonObstacle(points=poly_pts))

            planner = self._hybrid_module.AdaptiveHybridPlanner()
            result = planner.plan(
                start=start,
                goal=goal,
                obstacles=obstacles,
                safety_margin=max(0.0, float(getattr(task_config, "inflation_radius", 0.0))),
                turning_radius=max(0.1, float(getattr(task_config, "turn_radius", 0.8))),
                point_spacing=max(0.05, float(getattr(task_config, "default_path_spacing", 0.3))),
                resolution=max(0.05, min(0.5, resolution)),
            )
            if not bool(result.get("ok", False)):
                return []
            path = list(result.get("path", []) or [])
            if len(path) < 2:
                return []
            connector = []
            # Skip first point to avoid duplicate with previous segment end.
            for item in path[1:]:
                px = float(item["point"]["x"])
                py = float(item["point"]["y"])
                col = int(round((px - origin_x) / max(resolution, 1e-6)))
                row = int(round((py - origin_y) / max(resolution, 1e-6)))
                col = max(0, min(int(map_info["width"]) - 1, col))
                row = max(0, min(int(map_info["height"]) - 1, row))
                connector.append(module.Point(row=row, col=col))
            return connector
        except Exception as exc:
            rospy.logwarn("Hybrid connector planning failed, fallback to legacy connector: %s", exc)
            return []

    def _snap_point_to_free_cell(self, module, grid, point, max_radius=20):
        try:
            height, width = grid.shape[:2]
            row = max(0, min(int(height) - 1, int(round(float(point.row)))))
            col = max(0, min(int(width) - 1, int(round(float(point.col)))))
            if int(grid[row, col]) == 0:
                return module.Point(row=row, col=col)

            best = None
            best_dist = float("inf")
            for radius in range(1, max(1, int(max_radius)) + 1):
                r_min = max(0, row - radius)
                r_max = min(int(height) - 1, row + radius)
                c_min = max(0, col - radius)
                c_max = min(int(width) - 1, col + radius)

                for rr in range(r_min, r_max + 1):
                    for cc in (c_min, c_max):
                        if int(grid[rr, cc]) != 0:
                            continue
                        dist = (rr - row) * (rr - row) + (cc - col) * (cc - col)
                        if dist < best_dist:
                            best = (rr, cc)
                            best_dist = dist
                for cc in range(c_min + 1, c_max):
                    for rr in (r_min, r_max):
                        if int(grid[rr, cc]) != 0:
                            continue
                        dist = (rr - row) * (rr - row) + (cc - col) * (cc - col)
                        if dist < best_dist:
                            best = (rr, cc)
                            best_dist = dist

                if best is not None:
                    return module.Point(row=int(best[0]), col=int(best[1]))

            return module.Point(row=row, col=col)
        except Exception:
            return point

    def _build_straight_connector(self, module, start, goal):
        try:
            r0 = int(round(float(start.row)))
            c0 = int(round(float(start.col)))
            r1 = int(round(float(goal.row)))
            c1 = int(round(float(goal.col)))
            steps = max(abs(r1 - r0), abs(c1 - c0), 1)
            points = []
            for idx in range(0, steps + 1):
                t = float(idx) / float(steps)
                row = int(round(float(r0) + (float(r1) - float(r0)) * t))
                col = int(round(float(c0) + (float(c1) - float(c0)) * t))
                if points and int(points[-1].row) == row and int(points[-1].col) == col:
                    continue
                points.append(module.Point(row=row, col=col))
            return points
        except Exception:
            return []

    def _grid_point_in_any_work_region(self, row, col, work_regions, map_info, ratio):
        if not work_regions:
            return True
        for region in work_regions:
            points = region.get("points", [])
            if len(points) < 3:
                continue
            poly = np.array(
                [[
                    float((p["x"] - map_info["origin_x"]) * ratio),
                    float((p["y"] - map_info["origin_y"]) * ratio),
                ] for p in points],
                dtype=np.float32,
            )
            # cv2.pointPolygonTest expects (x, y) -> (col, row)
            if cv2.pointPolygonTest(poly, (float(col), float(row)), False) >= 0:
                return True
        return False

    def _grid_point_in_region(self, row, col, region, map_info, ratio):
        points = region.get("points", []) if isinstance(region, dict) else []
        if len(points) < 3:
            return False
        poly = np.array(
            [[
                float((p["x"] - map_info["origin_x"]) * ratio),
                float((p["y"] - map_info["origin_y"]) * ratio),
            ] for p in points],
            dtype=np.float32,
        )
        return cv2.pointPolygonTest(poly, (float(col), float(row)), False) >= 0

    def _world_point_in_region(self, x, y, region):
        points = region.get("points", []) if isinstance(region, dict) else []
        if len(points) < 3:
            return False
        poly = np.array(
            [[float(p["x"]), float(p["y"])] for p in points],
            dtype=np.float32,
        )
        return cv2.pointPolygonTest(poly, (float(x), float(y)), False) >= 0

    def _fallback_plan(self, task_id, composed_grid, map_info, task_config):
        resolution = map_info["resolution"]
        self._path_version += 1
        ros_path = NavPath()
        ros_path.header.frame_id = map_info["frame_id"] or "map"
        work_regions = task_config.work_regions
        if not work_regions:
            auto_regions = self._build_free_space_regions(composed_grid, map_info)
            if auto_regions:
                work_regions = auto_regions
                rospy.logwarn(
                    "Fallback planner: auto-derived %d work region(s) from free space contours.",
                    len(auto_regions),
                )
            else:
                work_regions = [self._build_full_map_region(composed_grid.shape, resolution, map_info)]
                rospy.logwarn("Fallback planner: free-space contour extraction failed, using full-map region.")
        points = []
        for region in work_regions:
            xs = [p["x"] for p in region["points"]]
            ys = [p["y"] for p in region["points"]]
            if not xs or not ys:
                continue
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            spacing = max(task_config.default_path_spacing, resolution * 2.0)
            y = min_y
            reverse = False
            while y <= max_y + 1e-6:
                x_pair = (max_x, min_x) if reverse else (min_x, max_x)
                for x in x_pair:
                    pose = PoseStamped()
                    pose.header.frame_id = ros_path.header.frame_id
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.orientation.w = 1.0
                    ros_path.poses.append(pose)
                    points.append({"x": x, "y": y, "path_type": region.get("name", "fallback")})
                reverse = not reverse
                y += spacing
        length_m = 0.0
        for index in range(len(points) - 1):
            length_m += math.hypot(points[index + 1]["x"] - points[index]["x"], points[index + 1]["y"] - points[index]["y"])
        for idx, item in enumerate(points):
            if idx + 1 < len(points):
                nxt = points[idx + 1]
                yaw = math.atan2(float(nxt["y"]) - float(item["y"]), float(nxt["x"]) - float(item["x"]))
            elif idx > 0:
                prv = points[idx - 1]
                yaw = math.atan2(float(item["y"]) - float(prv["y"]), float(item["x"]) - float(prv["x"]))
            else:
                yaw = 0.0
            item["orientation"] = self._yaw_to_quaternion(yaw)
        return PlannerPath(
            task_id=task_id,
            path_version=self._path_version,
            points=points,
            nav_path=ros_path,
            length_m=length_m,
        )

    def _build_full_map_region(self, shape, resolution, map_info):
        height, width = shape
        origin_x = map_info["origin_x"]
        origin_y = map_info["origin_y"]
        return {
            "name": "full_map",
            "points": [
                {"x": origin_x, "y": origin_y},
                {"x": origin_x + width * resolution, "y": origin_y},
                {"x": origin_x + width * resolution, "y": origin_y + height * resolution},
                {"x": origin_x, "y": origin_y + height * resolution},
            ],
        }

    def _build_free_space_regions(self, composed_grid, map_info):
        free_mask = np.where(composed_grid == 0, 255, 0).astype(np.uint8)
        if int(np.count_nonzero(free_mask)) < 20:
            return []
        contours, _ = cv2.findContours(free_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []

        resolution = float(map_info["resolution"])
        origin_x = float(map_info["origin_x"])
        origin_y = float(map_info["origin_y"])
        width = int(map_info["width"])
        height = int(map_info["height"])
        min_area = 50.0
        regions = []
        # Cover all meaningful free-space islands, not only the largest one.
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            perimeter = cv2.arcLength(contour, True)
            epsilon = max(1.0, 0.01 * perimeter)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if approx is None or len(approx) < 3:
                continue
            points = []
            for item in approx:
                col = int(item[0][0])
                row = int(item[0][1])
                col = max(0, min(width - 1, col))
                row = max(0, min(height - 1, row))
                points.append(
                    {
                        "x": origin_x + float(col) * resolution,
                        "y": origin_y + float(row) * resolution,
                    }
                )
            if len(points) >= 3:
                regions.append({"name": "auto_free_space_{}".format(len(regions) + 1), "points": points})
        return regions

    def _world_to_point(self, module, x, y, map_info, ratio):
        col = int(round((x - map_info["origin_x"]) * ratio))
        row = int(round((y - map_info["origin_y"]) * ratio))
        col = max(0, min(map_info["width"] - 1, col))
        row = max(0, min(map_info["height"] - 1, row))
        return module.Point(row=row, col=col)

    def _build_global_boundary(self, module, work_regions, map_info, ratio):
        all_points = []
        for region in work_regions:
            for point in region.get("points", []):
                p = self._world_to_point(module, point["x"], point["y"], map_info, ratio)
                all_points.append((p.col, p.row))
        if len(all_points) < 3:
            full_region = self._build_full_map_region(
                (int(map_info["height"]), int(map_info["width"])),
                float(map_info["resolution"]),
                map_info,
            )
            return [self._world_to_point(module, p["x"], p["y"], map_info, ratio) for p in full_region["points"]]
        pts = np.array(all_points, dtype=np.int32).reshape((-1, 1, 2))
        hull = cv2.convexHull(pts)
        boundary = []
        for item in hull:
            col = int(item[0][0])
            row = int(item[0][1])
            boundary.append(module.Point(row=row, col=col))
        return boundary

    def _apply_erase_regions_to_grid_map(self, grid_map, map_info, erase_regions):
        if grid_map is None or not erase_regions:
            return
        height = int(map_info["height"])
        width = int(map_info["width"])
        resolution = float(map_info["resolution"])
        origin_x = float(map_info["origin_x"])
        origin_y = float(map_info["origin_y"])
        # OpenCV drawing APIs are strict about image dtypes on some platforms.
        # Build a uint8 mask and then clear target cells in-place.
        erase_mask = np.zeros((height, width), dtype=np.uint8)
        for region in erase_regions:
            points = region.get("points", []) if isinstance(region, dict) else []
            if len(points) < 3:
                continue
            polygon = []
            for pt in points:
                col = int(round((float(pt["x"]) - origin_x) / max(resolution, 1e-6)))
                row = int(round((float(pt["y"]) - origin_y) / max(resolution, 1e-6)))
                col = max(0, min(width - 1, col))
                row = max(0, min(height - 1, row))
                polygon.append([col, row])
            if len(polygon) >= 3:
                cv2.fillPoly(erase_mask, [np.array(polygon, dtype=np.int32)], 255)
        if int(np.count_nonzero(erase_mask)) > 0:
            grid_map[erase_mask > 0] = 0

    def _apply_obstacle_regions_to_grid_map(self, grid_map, map_info, obstacle_regions):
        if grid_map is None or not obstacle_regions:
            return 0
        height, width = grid_map.shape[:2]
        resolution = float(map_info["resolution"])
        origin_x = float(map_info["origin_x"])
        origin_y = float(map_info["origin_y"])
        obstacle_mask = np.zeros((height, width), dtype=np.uint8)
        for region in obstacle_regions:
            if not isinstance(region, dict) or not bool(region.get("enabled", True)):
                continue
            points = region.get("points", []) or []
            if len(points) < 3:
                continue
            polygon = []
            for pt in points:
                col = int(round((float(pt["x"]) - origin_x) / max(resolution, 1e-6)))
                row = int(round((float(pt["y"]) - origin_y) / max(resolution, 1e-6)))
                col = max(0, min(width - 1, col))
                row = max(0, min(height - 1, row))
                polygon.append([col, row])
            cv2.fillPoly(obstacle_mask, [np.asarray(polygon, dtype=np.int32)], 255)
        mask = obstacle_mask > 0
        mask_cells = int(np.count_nonzero(mask))
        if mask_cells > 0:
            grid_map[mask] = 1
        return mask_cells

    def _apply_crop_region_to_grid_map(self, grid_map, map_info, crop_region):
        if grid_map is None or not isinstance(crop_region, dict):
            return
        points = crop_region.get("points", []) or []
        if len(points) < 3:
            return
        height = int(map_info["height"])
        width = int(map_info["width"])
        resolution = float(map_info["resolution"])
        origin_x = float(map_info["origin_x"])
        origin_y = float(map_info["origin_y"])
        mask = np.zeros((height, width), dtype=np.uint8)
        polygon = []
        for pt in points:
            col = int(round((float(pt["x"]) - origin_x) / max(resolution, 1e-6)))
            row = int(round((float(pt["y"]) - origin_y) / max(resolution, 1e-6)))
            col = max(0, min(width - 1, col))
            row = max(0, min(height - 1, row))
            polygon.append([col, row])
        if len(polygon) >= 3:
            cv2.fillPoly(mask, [np.array(polygon, dtype=np.int32)], 255)
            # Outside crop area is removed from planning space.
            grid_map[mask == 0] = 1
