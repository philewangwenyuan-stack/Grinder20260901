#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
修复版 v3.2
"""

import numpy as np
import matplotlib

matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
import heapq
import math
import json
from dataclasses import dataclass, asdict
from typing import List, Optional, Tuple
from matplotlib.path import Path as MplPath
from scipy import ndimage
from datetime import datetime


# ============================================================
# 基础数据结构
# ============================================================

@dataclass
class Point:
    row: int
    col: int

    def __eq__(self, other):
        return self.row == other.row and self.col == other.col

    def __hash__(self):
        return hash((self.row, self.col))


# ============================================================
# 坐标转换辅助函数
# ============================================================

def real_to_grid_point(real_x: float, real_y: float, real_to_grid_ratio: float) -> Point:
    """
    真实坐标转换为网格坐标

    Args:
        real_x: 真实世界的x坐标（米）
        real_y: 真实世界的y坐标（米）
        real_to_grid_ratio: 真实坐标到网格的转换比例
            - 例如：100x100网格表示50mx50m区域，ratio = 100/50 = 2
            - 即：每米对应2个网格单元

    Returns:
        Point: 网格坐标点
            - Point.row = 网格行索引（对应y轴）
            - Point.col = 网格列索引（对应x轴）

    Example:
        场景：50m x 50m 区域用 100x100 网格表示

        ratio = 2.0  # 每米2个网格
        p = real_to_grid_point(5.0, 10.0, ratio)
        # 结果: p.col=10, p.row=20
    """
    grid_col = int(round(real_x * real_to_grid_ratio))  # x → col
    grid_row = int(round(real_y * real_to_grid_ratio))  # y → row
    return Point(row=grid_row, col=grid_col)


def grid_to_real_point(grid_point: Point, real_to_grid_ratio: float) -> Tuple[float, float]:
    """
    网格坐标转换为真实坐标

    Args:
        grid_point: 网格坐标点
        real_to_grid_ratio: 真实坐标到网格的转换比例

    Returns:
        tuple: (real_x, real_y) 真实世界坐标（米）

    Example:
        ratio = 2.0
        grid_pt = Point(row=20, col=10)
        real_x, real_y = grid_to_real_point(grid_pt, ratio)
        # 结果: (5.0, 10.0)
    """
    real_x = grid_point.col / real_to_grid_ratio  # col → x
    real_y = grid_point.row / real_to_grid_ratio  # row → y
    return real_x, real_y


def create_polygon_from_real_coords(real_coords: List[Tuple[float, float]],
                                    real_to_grid_ratio: float) -> List[Point]:
    """
    从真实坐标列表创建网格多边形

    Args:
        real_coords: 真实坐标列表 [(x1,y1), (x2,y2), ...]
        real_to_grid_ratio: 转换比例

    Returns:
        List[Point]: 网格坐标点列表

    Example:
        真实世界的矩形区域转换为网格坐标

        real_rect = [(0, 0), (10, 0), (10, 20), (0, 20)]
        ratio = 2.0
        grid_poly = create_polygon_from_real_coords(real_rect, ratio)
        # 结果: 返回4个Point对象的列表
    """
    return [real_to_grid_point(x, y, real_to_grid_ratio) for x, y in real_coords]


@dataclass
class PathPoint:
    row: float
    col: float
    time: float = 0.0
    point_type: str = 'intermediate'
    path_type: str = ''


@dataclass
class StageConfig:
    stage_id: int
    boundary_polygon: List[Point]
    start_point: Optional[Point]
    end_point: Point
    direction: str
    path_spacing: float
    obstacle_polygons: Optional[List[List[Point]]] = None
    direction_angle: Optional[float] = None


@dataclass
class RobotConfig:
    width: float = 1.3
    length: float = 2.0
    path_spacing: float = 1.17
    turning_radius: float = 0.8
    overlap_ratio: float = 0.1
    inflation_radius: float = 0.65  # 【修改】从0.3改为0.65 (width/2)


# ============================================================
# 障碍物处理器
# ============================================================

class ObstacleProcessor:
    @staticmethod
    def inflate_obstacles(grid_map: np.ndarray, inflation_radius_pixels: int) -> np.ndarray:
        if inflation_radius_pixels <= 0:
            return grid_map.copy()

        diameter = 2 * inflation_radius_pixels + 1
        y, x = np.ogrid[-inflation_radius_pixels:inflation_radius_pixels + 1,
               -inflation_radius_pixels:inflation_radius_pixels + 1]
        kernel = x * x + y * y <= inflation_radius_pixels * inflation_radius_pixels

        inflated = ndimage.binary_dilation(grid_map, structure=kernel).astype(int)

        print(f"   [障碍物膨胀] 膨胀半径: {inflation_radius_pixels}像素, "
              f"原始: {np.sum(grid_map)}, 膨胀后: {np.sum(inflated)}")

        return inflated

    @staticmethod
    def polygon_to_grid(polygon_points: List[Point], grid_shape: Tuple[int, int]) -> np.ndarray:
        """
        将多边形障碍物转换为grid格式
        """
        grid = np.zeros(grid_shape, dtype=int)
        if not polygon_points:
            return grid

        poly_verts = [(p.col, p.row) for p in polygon_points]
        ny, nx = grid_shape
        x, y = np.meshgrid(np.arange(nx), np.arange(ny))
        x, y = x.flatten(), y.flatten()
        points = np.vstack((x, y)).T
        path = MplPath(poly_verts)
        mask = path.contains_points(points).reshape((ny, nx))
        grid[mask] = 1

        return grid


# ============================================================
# 路径平滑器
# ============================================================

class PathSmoother:
    """
    不进行平滑，保持弓字形路径的原始形状
    """

    @staticmethod
    def smooth_path(path: List[Point], turning_radius: float,
                    grid_map: np.ndarray) -> List[PathPoint]:
        """不进行平滑，直接返回原始路径"""
        print(f"   [路径处理] 保持原始路径，点数: {len(path)}")
        return [PathPoint(float(p.row), float(p.col)) for p in path]


# ============================================================
# 覆盖率校验器
# ============================================================

class CoverageValidator:
    @staticmethod
    def check_full_coverage(map_size, global_polygon, sub_regions):
        rows, cols = map_size
        global_mask = np.zeros((rows, cols), dtype=bool)
        CoverageValidator._fill_polygon(global_mask, global_polygon)

        sub_mask_sum = np.zeros((rows, cols), dtype=bool)
        for poly in sub_regions:
            temp_mask = np.zeros((rows, cols), dtype=bool)
            CoverageValidator._fill_polygon(temp_mask, poly)
            sub_mask_sum = np.logical_or(sub_mask_sum, temp_mask)

        missing_areas = np.logical_and(global_mask, np.logical_not(sub_mask_sum))
        missing_count = np.sum(missing_areas)
        total_area = np.sum(global_mask)

        print(f"   [覆盖校验] 总面积: {total_area}, 遗漏: {missing_count}")
        return missing_count <= total_area * 0.01

    @staticmethod
    def _fill_polygon(mask, polygon_points):
        if not polygon_points: return
        poly_verts = [(p.col, p.row) for p in polygon_points]
        ny, nx = mask.shape
        x, y = np.meshgrid(np.arange(nx), np.arange(ny))
        x, y = x.flatten(), y.flatten()
        points = np.vstack((x, y)).T
        path = MplPath(poly_verts)
        grid = path.contains_points(points).reshape((ny, nx))
        mask[grid] = True


# ============================================================
# 核心弓字形规划器
# ============================================================

class ObstacleAwareBoustrophedonPlanner:
    def __init__(self, robot_config: RobotConfig):
        self.robot_config = robot_config

    def _is_point_in_polygon(self, point: Point, polygon: List[Point], on_boundary_counts=True) -> bool:
        """
        判断点是否在多边形内部

        Args:
            point: 待判断的点
            polygon: 多边形顶点列表
            on_boundary_counts: 边界上的点是否算作"在内部" (默认True)
        """
        poly_verts = [(p.col, p.row) for p in polygon]
        path = MplPath(poly_verts)

        # 【修复】使用radius=0.0进行严格判断，避免边界外的点被误判为内部
        if on_boundary_counts:
            return path.contains_point((point.col, point.row), radius=0.0)
        else:
            # 边界内部至少0.5像素才算
            return path.contains_point((point.col, point.row), radius=-0.5)

    def _find_nearest_free_point(self, grid_map: np.ndarray, p: Point) -> Point:
        rows, cols = grid_map.shape
        if 0 <= p.row < rows and 0 <= p.col < cols and grid_map[p.row, p.col] == 0:
            return p

        queue = [(p.row, p.col)]
        visited = set([(p.row, p.col)])
        max_search = 1000
        count = 0

        while queue and count < max_search:
            r, c = queue.pop(0)
            if 0 <= r < rows and 0 <= c < cols and grid_map[r, c] == 0:
                return Point(r, c)

            for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append((nr, nc))
            count += 1

        return p

    def _find_farthest_corner(self, polygon: List[Point], reference_point: Point) -> Point:
        if not polygon:
            return reference_point

        max_dist = -1
        farthest_point = polygon[0]

        for point in polygon:
            dist = np.sqrt((point.row - reference_point.row) ** 2 +
                           (point.col - reference_point.col) ** 2)
            if dist > max_dist:
                max_dist = dist
                farthest_point = point

        print(f"   [起点选择] 最远角点: ({farthest_point.row}, {farthest_point.col}), 距离: {max_dist:.2f}")
        return farthest_point

    def plan_subregion_boustrophedon(self, grid_map, boundary_polygon, direction,
                                     path_spacing, direction_angle=None,
                                     start_point=None, end_point=None):

        rows = [p.row for p in boundary_polygon]
        cols = [p.col for p in boundary_polygon]
        if not rows: return []
        min_row, max_row = max(0, min(rows)), min(grid_map.shape[0] - 1, max(rows))
        min_col, max_col = max(0, min(cols)), min(grid_map.shape[1] - 1, max(cols))

        all_segments = []
        is_x = direction.lower() == 'x'

        if is_x:
            # ========== 横向扫描（固定row，沿col扫描）==========

            # 生成扫描线位置
            if start_point:
                start_row = start_point.row
            else:
                start_row = min_row

            if end_point:
                end_row = end_point.row
            else:
                end_row = max_row

            scan_lines = []
            if start_row <= end_row:
                current_row = start_row
                while current_row <= end_row:
                    scan_lines.append(current_row)
                    current_row += path_spacing
                if scan_lines and scan_lines[-1] != end_row:
                    if end_row - scan_lines[-1] > path_spacing / 2:
                        scan_lines.append(end_row)
            else:
                current_row = start_row
                while current_row >= end_row:
                    scan_lines.append(current_row)
                    current_row -= path_spacing
                if scan_lines and scan_lines[-1] != end_row:
                    if scan_lines[-1] - end_row > path_spacing / 2:
                        scan_lines.append(end_row)

            print(f"   [扫描线] 横向扫描: 共{len(scan_lines)}条")

            for i, row in enumerate(scan_lines):
                scan_start_col = min_col
                scan_end_col = max_col

                # 判断这条扫描线是否会被反向
                will_reverse = (i % 2 == 1)

                # 第一条扫描线：从起点开始
                if i == 0 and start_point:
                    if not will_reverse:
                        # 不反向：正常设置起点
                        scan_start_col = start_point.col
                    else:
                        # 会反向：起点要设置在end位置（反向后变成起点）
                        scan_end_col = start_point.col
                    print(f"   [扫描线{i}] 第一条，从起点 col={start_point.col} 开始")

                # 在终点结束
                if i == len(scan_lines) - 1 and end_point:
                    if not will_reverse:
                        # 不反向：正常设置终点
                        scan_end_col = end_point.col
                    else:
                        # 会反向：终点要设置在start位置（反向后变成终点）
                        scan_start_col = end_point.col
                    print(f"   [扫描线{i}] 最后一条，在终点 col={end_point.col} 结束 (反向={will_reverse})")

                segments_in_row = self._get_scan_segments(
                    grid_map, boundary_polygon, row,
                    scan_start_col, scan_end_col, True
                )

                if i % 2 == 1:
                    segments_in_row.reverse()
                    for seg in segments_in_row:
                        seg.reverse()
                all_segments.extend(segments_in_row)

        else:
            # ========== 纵向扫描（固定col，沿row扫描）==========

            if start_point:
                start_col = start_point.col
            else:
                start_col = min_col

            if end_point:
                end_col = end_point.col
            else:
                end_col = max_col

            scan_lines = []
            if start_col <= end_col:
                current_col = start_col
                while current_col <= end_col:
                    scan_lines.append(current_col)
                    current_col += path_spacing
                if scan_lines and scan_lines[-1] != end_col:
                    if end_col - scan_lines[-1] > path_spacing / 2:
                        scan_lines.append(end_col)
            else:
                current_col = start_col
                while current_col >= end_col:
                    scan_lines.append(current_col)
                    current_col -= path_spacing
                if scan_lines and scan_lines[-1] != end_col:
                    if scan_lines[-1] - end_col > path_spacing / 2:
                        scan_lines.append(end_col)

            print(f"   [扫描线] 纵向扫描: 共{len(scan_lines)}条")

            # 控制每条扫描线内部的起止点
            for i, col in enumerate(scan_lines):
                scan_start_row = min_row
                scan_end_row = max_row

                will_reverse = (i % 2 == 1)

                if i == 0 and start_point:
                    if not will_reverse:
                        scan_start_row = start_point.row
                    else:
                        scan_end_row = start_point.row
                    print(f"   [扫描线{i}] 第一条，从起点 row={start_point.row} 开始")

                if i == len(scan_lines) - 1 and end_point:
                    if not will_reverse:
                        scan_end_row = end_point.row
                    else:
                        scan_start_row = end_point.row
                    print(f"   [扫描线{i}] 最后一条，在终点 row={end_point.row} 结束 (反向={will_reverse})")

                segments_in_col = self._get_scan_segments(
                    grid_map, boundary_polygon, col,
                    scan_start_row, scan_end_row, False
                )

                if i % 2 == 1:
                    segments_in_col.reverse()
                    for seg in segments_in_col:
                        seg.reverse()
                all_segments.extend(segments_in_col)

        # 拼接线段
        full_path = []
        if not all_segments:
            return []

        full_path.extend(all_segments[0])

        for i in range(1, len(all_segments)):
            prev_seg = all_segments[i - 1]
            curr_seg = all_segments[i]
            start_node = prev_seg[-1]
            end_node = curr_seg[0]
            dist = abs(start_node.row - end_node.row) + abs(start_node.col - end_node.col)

            if dist > 1.5:
                bridge_path = self._astar_search(grid_map, start_node, end_node)
                if bridge_path and len(bridge_path) > 1:
                    full_path.extend(bridge_path[1:-1])
                full_path.extend(curr_seg)
            else:
                if len(curr_seg) > 0:
                    full_path.extend(curr_seg)

        # 【新增】后处理：严格过滤边界外的点
        filtered_path = []
        for point in full_path:
            # 检查是否在网格范围内
            if not (0 <= point.row < grid_map.shape[0] and 0 <= point.col < grid_map.shape[1]):
                continue
            # 严格检查是否在多边形内部
            if self._is_point_in_polygon(point, boundary_polygon, on_boundary_counts=False):
                filtered_path.append(point)

        if len(filtered_path) < len(full_path):
            print(f"   [边界过滤] 原始: {len(full_path)} 点, 过滤后: {len(filtered_path)} 点, "
                  f"移除: {len(full_path) - len(filtered_path)} 个越界点")

        # 终点保证机制
        if end_point and filtered_path:
            last_point = filtered_path[-1]
            dist_to_end = abs(last_point.row - end_point.row) + abs(last_point.col - end_point.col)

            if dist_to_end > 1:
                if (0 <= end_point.row < grid_map.shape[0] and
                        0 <= end_point.col < grid_map.shape[1] and
                        grid_map[end_point.row, end_point.col] == 0 and
                        self._is_point_in_polygon(end_point, boundary_polygon, on_boundary_counts=False)):
                    filtered_path.append(end_point)
                    print(f"   [终点添加] 终点已添加到路径末尾")

        return filtered_path

    def _get_scan_segments(self, grid_map, poly, fixed_idx, start_idx, end_idx, is_row_scan):
        """
        【修复版】获取扫描线段，增加严格的边界检查
        """
        segments = []
        current_segment = []

        for moving_idx in range(start_idx, end_idx + 1):
            r, c = (fixed_idx, moving_idx) if is_row_scan else (moving_idx, fixed_idx)

            # 检查是否在网格范围内
            if not (0 <= r < grid_map.shape[0] and 0 <= c < grid_map.shape[1]):
                if current_segment:
                    segments.append(current_segment)
                    current_segment = []
                continue

            # 检查是否有障碍物 + 严格的多边形内部检查
            is_valid = (grid_map[r, c] == 0) and self._is_point_in_polygon(Point(r, c), poly, on_boundary_counts=False)

            if is_valid:
                current_segment.append(Point(r, c))
            else:
                if current_segment:
                    segments.append(current_segment)
                    current_segment = []

        if current_segment:
            segments.append(current_segment)

        return segments

    def plan_connection_path(self, grid_map, start, goal):
        safe_start = self._find_nearest_free_point(grid_map, start)
        safe_goal = self._find_nearest_free_point(grid_map, goal)
        return self._astar_search(grid_map, safe_start, safe_goal)

    def _astar_search(self, grid_map, start, goal):
        start_node = (start.row, start.col)
        goal_node = (goal.row, goal.col)
        if start_node == goal_node: return [start]

        rows, cols = grid_map.shape
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0}

        movements = [
            (0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        max_steps = 50000
        steps = 0

        while open_set and steps < max_steps:
            steps += 1
            current = heapq.heappop(open_set)[1]

            if current == goal_node:
                path = []
                while current in came_from:
                    path.append(Point(current[0], current[1]))
                    current = came_from[current]
                path.append(Point(current[0], current[1]))
                return path[::-1]

            for dr, dc, cost in movements:
                nr, nc = current[0] + dr, current[1] + dc

                if 0 <= nr < rows and 0 <= nc < cols:
                    if grid_map[nr, nc] == 1: continue

                    if abs(dr) == 1 and abs(dc) == 1:
                        if grid_map[current[0] + dr, current[1]] == 1 or grid_map[current[0], current[1] + dc] == 1:
                            continue

                    new_g = g_score[current] + cost
                    neighbor = (nr, nc)
                    if neighbor not in g_score or new_g < g_score[neighbor]:
                        g_score[neighbor] = new_g
                        priority = new_g + abs(nr - goal_node[0]) + abs(nc - goal_node[1])
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        print(f"   [A*警告] 未找到路径: {start} -> {goal}")
        return [start, goal]


# ============================================================
# JSON输出功能
# ============================================================

def export_path_to_json(path, filename='path_output.json', output_dir=None):
    """
    导出路径点为JSON格式，包含每个点的朝向（四元数，float64）
    """
    import math

    path_data = {
        "metadata": {
            "total_points": len(path),
            "export_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "coordinate_unit": "grid_pixels",
            "coordinate_system": "row=y, col=x",
            "description": "Boustrophedon path planning with boundary fix and orientation",
            "version": "3.3-fixed-orientation",
            "orientation_type": "quaternion (x,y,z,w) float64, heading toward next point",
            "fix_notes": "Added orientation (quaternion) for each point based on path direction"
        },
        "path": []
    }

    stage_counts = {}
    connection_count = 0

    # 预计算所有点的朝向
    orientations = []
    for i in range(len(path)):
        if i < len(path) - 1:
            p1 = path[i]
            p2 = path[i + 1]
            dx = p2.col - p1.col
            dy = p2.row - p1.row
            yaw = math.atan2(dy, dx)          # 偏航角，弧度
            half_yaw = yaw / 2.0
            orientations.append({
                'x': 0.0,
                'y': 0.0,
                'z': math.sin(half_yaw),      # float64
                'w': math.cos(half_yaw)       # float64
            })
        else:
            # 最后一个点沿用前一个点的朝向；若只有1个点则朝向 +x
            if orientations:
                orientations.append(orientations[-1].copy())
            else:
                orientations.append({
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'w': 1.0
                })

    # 组装路径点
    for i, point in enumerate(path):
        point_dict = {
            "index": i,
            "row": round(float(point.row), 3),
            "col": round(float(point.col), 3),
            "x": round(float(point.col), 3),
            "y": round(float(point.row), 3),
            "path_type": point.path_type,
            "point_type": point.point_type,
            "timestamp": point.time,
            "orientation": orientations[i]   # 四元数字典，值均为 float64
        }
        path_data["path"].append(point_dict)

        if point.path_type == 'connection':
            connection_count += 1
        else:
            stage_counts[point.path_type] = stage_counts.get(point.path_type, 0) + 1

    # 统计信息
    total_distance = 0.0
    for i in range(len(path) - 1):
        dx = path[i + 1].col - path[i].col
        dy = path[i + 1].row - path[i].row
        total_distance += np.sqrt(dx ** 2 + dy ** 2)

    path_data["statistics"] = {
        "stages": stage_counts,
        "connection_points": connection_count,
        "stage_count": len(stage_counts),
        "total_distance": round(total_distance, 2),
        "average_point_spacing": round(total_distance / (len(path) - 1) if len(path) > 1 else 0, 3)
    }

    if path:
        path_data["start_point"] = {
            "index": 0,
            "x": round(float(path[0].col), 3),
            "y": round(float(path[0].row), 3),
            "row": round(float(path[0].row), 3),
            "col": round(float(path[0].col), 3)
        }
        path_data["end_point"] = {
            "index": len(path) - 1,
            "x": round(float(path[-1].col), 3),
            "y": round(float(path[-1].row), 3),
            "row": round(float(path[-1].row), 3),
            "col": round(float(path[-1].col), 3)
        }

    # 保存文件
    import os
    if output_dir:
        if not os.path.exists(output_dir):
            try:
                os.makedirs(output_dir, exist_ok=True)
                print(f"   [创建目录] {output_dir}")
            except:
                print(f"   [警告] 无法创建目录 {output_dir}，使用当前目录")
                output_dir = None
        full_path = os.path.join(output_dir, filename)
    else:
        full_path = filename

    with open(full_path, 'w', encoding='utf-8') as f:
        json.dump(path_data, f, indent=2, ensure_ascii=False)

    print(f"\n{'=' * 60}")
    print(f"✅ 路径已导出为JSON格式: {full_path}")
    print(f"{'=' * 60}")
    print(f"路径统计：")
    print(f"  总点数: {len(path)}")
    print(f"  总路径长度: {total_distance:.2f} 像素")
    print(f"  平均点间距: {path_data['statistics']['average_point_spacing']} 像素")
    print(f"  阶段数: {len(stage_counts)}")
    for stage, count in stage_counts.items():
        print(f"    {stage}: {count} 点")
    print(f"  连接段点数: {connection_count}")
    if path:
        print(f"\n  起点: (x={path[0].col:.1f}, y={path[0].row:.1f})")
        print(f"  终点: (x={path[-1].col:.1f}, y={path[-1].row:.1f})")
    print(f"{'=' * 60}\n")

    return path_data


# ============================================================
# 多阶段规划器（带参数打印）
# ============================================================

class MultiStagePathPlanner:
    def __init__(self, robot_config: RobotConfig):
        self.robot_config = robot_config
        self.planner_core = ObstacleAwareBoustrophedonPlanner(robot_config)
        self.path_smoother = PathSmoother()

        # 安全检查
        self._validate_robot_config()

    def _validate_robot_config(self):
        """验证机器人配置的安全性"""
        min_inflation = self.robot_config.width / 2
        if self.robot_config.inflation_radius < min_inflation:
            print(f"\n⚠️  警告: 膨胀半径({self.robot_config.inflation_radius}m) < 最小建议值({min_inflation}m)")
            print(f"   建议设置: inflation_radius >= {min_inflation}m")

    def plan(self, grid_map, global_work_area, stages, real_to_grid_ratio,
             global_start_point=None, global_end_point=None,
             global_direction='x', global_path_spacing=None,
             global_obstacle_polygons=None):
        """
        多阶段路径规划
        """

        # ============================================================
        # 打印所有输入参数到控制台
        # ============================================================
        print("\n" + "=" * 80)
        print("输入参数详情")
        print("=" * 80)

        # 1. grid_map参数
        print(f"\n【1. grid_map】")
        print(f"  ├─ 类型: {type(grid_map).__name__}")
        if hasattr(grid_map, 'shape'):
            print(f"  ├─ 形状: {grid_map.shape} (高×宽 = {grid_map.shape[0]}×{grid_map.shape[1]})")
            print(f"  ├─ 数据类型: {grid_map.dtype}")
            print(f"  ├─ 障碍物格子数: {np.sum(grid_map)}")
            print(f"  └─ 可用格子数: {np.sum(grid_map == 0)}")
        else:
            print(f"  └─ ⚠️  非numpy数组")

        # 2. global_work_area参数
        print(f"\n【2. global_work_area】")
        print(f"  ├─ 类型: {type(global_work_area).__name__}")
        print(f"  ├─ 顶点数量: {len(global_work_area) if global_work_area else 0}")
        if global_work_area and len(global_work_area) > 0:
            print(f"  ├─ 顶点列表 (前5个):")
            for i, p in enumerate(global_work_area[:min(5, len(global_work_area))]):
                prefix = "  │   ├─" if i < min(4, len(global_work_area) - 1) else "  │   └─"
                print(f"{prefix} [{i}] Point(row={p.row}, col={p.col})")
            if len(global_work_area) > 5:
                print(f"  │   ... (共{len(global_work_area)}个顶点)")
            # 计算边界框
            rows = [p.row for p in global_work_area]
            cols = [p.col for p in global_work_area]
            print(f"  └─ 边界框: row[{min(rows)}, {max(rows)}], col[{min(cols)}, {max(cols)}]")
        else:
            print(f"  └─ ️  空列表或None")

        # 3. stages参数
        print(f"\n【3. stages】")
        print(f"  ├─ 类型: {type(stages).__name__}")
        print(f"  ├─ 阶段数量: {len(stages) if stages else 0}")
        if stages and len(stages) > 0:
            for idx, stage in enumerate(stages):
                is_last = (idx == len(stages) - 1)
                prefix = "  └─" if is_last else "  ├─"
                print(f"{prefix} Stage {stage.stage_id}:")
                sub_prefix = "     " if is_last else "  │  "
                print(f"{sub_prefix}├─ boundary顶点数: {len(stage.boundary_polygon)}")
                print(f"{sub_prefix}├─ start_point: {stage.start_point if stage.start_point else 'None (自动)'}")
                print(f"{sub_prefix}├─ end_point: {stage.end_point}")
                print(f"{sub_prefix}├─ direction: '{stage.direction}'")
                print(f"{sub_prefix}├─ path_spacing: {stage.path_spacing}")
                obs_count = len(stage.obstacle_polygons) if stage.obstacle_polygons else 0
                print(f"{sub_prefix}└─ 障碍物数量: {obs_count}")
        else:
            print(f"  └─ 空列表 (将使用全局模式)")

        # 4. real_to_grid_ratio参数
        print(f"\n【4. real_to_grid_ratio】")
        print(f"  ├─ 值: {real_to_grid_ratio}")
        print(f"  ├─ 类型: {type(real_to_grid_ratio).__name__}")
        print(f"  ├─ 含义: 1米 = {real_to_grid_ratio:.3f} 像素")
        print(f"  └─ 反向: 1像素 = {1 / real_to_grid_ratio:.3f} 米")

        # 5. global_start_point参数
        print(f"\n【5. global_start_point】")
        if global_start_point:
            print(f"  ├─ 值: Point(row={global_start_point.row}, col={global_start_point.col})")
            print(f"  └─ 类型: {type(global_start_point).__name__}")
        else:
            print(f"  └─ None (将自动选择)")

        # 6. global_end_point参数
        print(f"\n【6. global_end_point】")
        if global_end_point:
            print(f"  ├─ 值: Point(row={global_end_point.row}, col={global_end_point.col})")
            print(f"  └─ 类型: {type(global_end_point).__name__}")
        else:
            print(f"  └─ None (将自动选择)")

        # 7. global_direction参数
        print(f"\n【7. global_direction】")
        print(f"  ├─ 值: '{global_direction}'")
        print(f"  ├─ 类型: {type(global_direction).__name__}")
        direction_desc = "横向扫描(固定row,沿col移动)" if global_direction.lower() == 'x' else "纵向扫描(固定col,沿row移动)"
        print(f"  └─ 含义: {direction_desc}")

        # 8. global_path_spacing参数
        print(f"\n【8. global_path_spacing】")
        if global_path_spacing is not None:
            print(f"  ├─ 值: {global_path_spacing} 像素")
            real_spacing = global_path_spacing / real_to_grid_ratio
            print(f"  ├─ 对应真实距离: {real_spacing:.3f} 米")
            print(f"  └─ 类型: {type(global_path_spacing).__name__}")
        else:
            print(f"  └─ None (将使用robot_config默认值)")

        # 9. global_obstacle_polygons参数
        print(f"\n【9. global_obstacle_polygons】")
        if global_obstacle_polygons:
            print(f"  ├─ 障碍物数量: {len(global_obstacle_polygons)}")
            for i, obs in enumerate(global_obstacle_polygons):
                is_last = (i == len(global_obstacle_polygons) - 1)
                prefix = "  └─" if is_last else "  ├─"
                print(f"{prefix} 障碍物[{i}]:")
                sub_prefix = "     " if is_last else "  │  "
                print(f"{sub_prefix}├─ 顶点数: {len(obs)}")
                if obs and len(obs) > 0:
                    print(f"{sub_prefix}├─ 第1个点: Point(row={obs[0].row}, col={obs[0].col})")
                    if len(obs) > 1:
                        print(f"{sub_prefix}└─ 最后点: Point(row={obs[-1].row}, col={obs[-1].col})")
        else:
            print(f"  └─ None (无全局障碍物)")

        # 10. 机器人配置
        print(f"\n【10. robot_config (当前实例配置)】")
        print(f"  ├─ width: {self.robot_config.width}m")
        print(f"  ├─ length: {self.robot_config.length}m")
        print(f"  ├─ path_spacing: {self.robot_config.path_spacing}m")
        print(f"  ├─ turning_radius: {self.robot_config.turning_radius}m")
        print(f"  ├─ overlap_ratio: {self.robot_config.overlap_ratio}")
        print(f"  └─ inflation_radius: {self.robot_config.inflation_radius}m")

        print("=" * 80)
        print("║ 参数打印完毕，开始执行规划...")
        print("=" * 80 + "\n")

        # ============================================================
        # 原有的函数逻辑从这里开始
        # ============================================================

        print("\n" + "=" * 60)
        print("开始路径规划（修复版 v3.2）")
        print("=" * 60)

        # 支持无子区域模式
        if not stages or len(stages) == 0:
            print("\n[模式] 无子区域定义，规划整个全局区域")

            if global_start_point is None:
                global_start_point = global_work_area[0]
                print(f"  → 起点: 自动选择全局区域第一个点 (x={global_start_point.col}, y={global_start_point.row})")
            else:
                print(f"  → 起点: 用户指定 (x={global_start_point.col}, y={global_start_point.row})")

            if global_end_point is None:
                if len(global_work_area) >= 3:
                    global_end_point = global_work_area[2]
                else:
                    global_end_point = global_work_area[-1]
                print(f"  → 终点: 自动选择对角点 (x={global_end_point.col}, y={global_end_point.row})")
            else:
                print(f"  → 终点: 用户指定 (x={global_end_point.col}, y={global_end_point.row})")

            print(f"  → 扫描方向: {global_direction}")

            if global_path_spacing is None:
                spacing = self.robot_config.path_spacing * real_to_grid_ratio
                print(f"  → 路径间距: 使用默认值 {spacing:.2f} 像素")
            else:
                spacing = global_path_spacing
                print(f"  → 路径间距: 用户指定 {spacing:.2f} 像素")

            if global_obstacle_polygons:
                print(f"  → 障碍物: {len(global_obstacle_polygons)} 个")
            else:
                print(f"  → 障碍物: 无")

            stages = [
                StageConfig(
                    stage_id=1,
                    boundary_polygon=global_work_area,
                    start_point=global_start_point,
                    end_point=global_end_point,
                    direction=global_direction,
                    path_spacing=spacing,
                    obstacle_polygons=global_obstacle_polygons
                )
            ]
            print(f"  ✓ 自动创建阶段1，覆盖整个全局区域")
        else:
            print(f"\n[模式] 多阶段规划，共{len(stages)}个子区域")

        # 处理障碍物
        print("\n[障碍物处理]")
        total_stage_obstacles = 0
        for stage in stages:
            if stage.obstacle_polygons:
                num_obstacles = len(stage.obstacle_polygons)
                total_stage_obstacles += num_obstacles
                print(f"  阶段{stage.stage_id}: {num_obstacles}个障碍物多边形")

                for i, obstacle_poly in enumerate(stage.obstacle_polygons):
                    obstacle_grid = ObstacleProcessor.polygon_to_grid(
                        obstacle_poly,
                        grid_map.shape
                    )
                    grid_map = np.logical_or(grid_map, obstacle_grid).astype(int)
                    print(f"    障碍物{i + 1}: {len(obstacle_poly)}个顶点 → {np.sum(obstacle_grid)}格")

        if total_stage_obstacles > 0:
            print(f"  ✓ 共处理 {total_stage_obstacles} 个障碍物多边形")
        else:
            print(f"  无阶段特定障碍物")

        inflation_pixels = int(self.robot_config.inflation_radius * real_to_grid_ratio)
        print(f"\n[参数] 机器人宽度: {self.robot_config.width}m, 长度: {self.robot_config.length}m")
        print(f"[参数] 膨胀半径: {self.robot_config.inflation_radius}m ({inflation_pixels}像素)")

        inflated_grid = ObstacleProcessor.inflate_obstacles(grid_map, inflation_pixels)

        effective_spacing = self.robot_config.path_spacing * (1 - self.robot_config.overlap_ratio)
        print(f"[参数] 路径间距: {self.robot_config.path_spacing}m, 重叠: {self.robot_config.overlap_ratio}")
        print(f"[参数] 有效间距: {effective_spacing:.2f}m")

        print(f"[说明] 仅规划已定义的子区域，无需完全覆盖全局区域")

        complete_path = []
        last_stage_end = None

        for i, stage in enumerate(stages):
            print(f"\n--- 阶段 {stage.stage_id} ---")

            # 起点选择
            if stage.start_point:
                curr_start = stage.start_point
                print(f"[起点] 使用指定起点: (x={curr_start.col}, y={curr_start.row})")
            else:
                curr_start = self.planner_core._find_farthest_corner(stage.boundary_polygon, stage.end_point)
                print(f"[起点] 自动选择最远角点: (x={curr_start.col}, y={curr_start.row})")

            curr_start = self.planner_core._find_nearest_free_point(inflated_grid, curr_start)

            # 阶段连接
            if last_stage_end:
                print(f"[连接] 阶段{i} → 阶段{i + 1}")
                print(f"  从: (x={last_stage_end.col}, y={last_stage_end.row})")
                print(f"  到: (x={curr_start.col}, y={curr_start.row})")

                dist = abs(last_stage_end.row - curr_start.row) + abs(last_stage_end.col - curr_start.col)
                print(f"  曼哈顿距离: {dist}像素")

                if dist > 2:
                    conn_pts = self.planner_core.plan_connection_path(inflated_grid, last_stage_end, curr_start)
                    connection_segment = conn_pts[1:-1] if len(conn_pts) > 2 else []

                    if connection_segment:
                        print(f"  连接段: {len(connection_segment)} 个中间点")
                        for p in connection_segment:
                            complete_path.append(PathPoint(float(p.row), float(p.col), 0, 'intermediate', 'connection'))
                    else:
                        print(f"  直接连接（无中间点）")
                else:
                    print(f"  距离太近，无需连接段")

            spacing_pixels = max(1, int(round(effective_spacing * real_to_grid_ratio)))
            print(f"[规划] 间距: {spacing_pixels}像素, 方向: {stage.direction}")

            region_pts = self.planner_core.plan_subregion_boustrophedon(
                inflated_grid,
                stage.boundary_polygon,
                stage.direction,
                spacing_pixels,
                stage.direction_angle,
                start_point=curr_start,
                end_point=stage.end_point
            )

            if region_pts:
                for p in region_pts:
                    complete_path.append(
                        PathPoint(float(p.row), float(p.col), 0, 'intermediate', f'stage_{stage.stage_id}'))

                last_stage_end = region_pts[-1]
                print(f"[完成] 路径点数: {len(region_pts)}")
            else:
                last_stage_end = curr_start
                print(f"[警告] 区域无有效路径")

        print("\n" + "=" * 60)
        print(f"规划完成！总路径点数: {len(complete_path)}")
        print("=" * 60 + "\n")

        return complete_path


# ============================================================
# 可视化函数
# ============================================================

def visualize_path(grid, global_poly, stages, path, robot_config, output_dir=None, filename='path_planning_fixed.png'):
    """
    显示每个阶段的起终点
    """
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))

    # 处理障碍物
    full_grid = grid.copy()
    for stage in stages:
        if stage.obstacle_polygons:
            for obstacle_poly in stage.obstacle_polygons:
                obstacle_grid = ObstacleProcessor.polygon_to_grid(obstacle_poly, grid.shape)
                full_grid = np.logical_or(full_grid, obstacle_grid).astype(int)

    inflation_pixels = int(robot_config.inflation_radius * 1.0)
    inflated_grid = ObstacleProcessor.inflate_obstacles(full_grid, inflation_pixels)

    # 左图
    ax1 = axes[0]
    ax1.imshow(full_grid, cmap='Greys', origin='lower', alpha=0.7)
    ax1.set_title('Original Map with Obstacles', fontsize=14, fontweight='bold')

    # 右图
    ax2 = axes[1]
    ax2.imshow(inflated_grid, cmap='Greys', origin='lower', alpha=0.5)
    ax2.set_title('Path Planning Result - FIXED v3.2', fontsize=14, fontweight='bold')

    # 绘制边界
    for ax in axes:
        gx, gy = zip(*[(p.col, p.row) for p in global_poly] + [(global_poly[0].col, global_poly[0].row)])
        ax.plot(gx, gy, 'g-', linewidth=3, alpha=0.3, label='Global Area')

        colors = ['blue', 'orange', 'red', 'purple']
        for i, s in enumerate(stages):
            sx, sy = zip(*[(p.col, p.row) for p in s.boundary_polygon] +
                          [(s.boundary_polygon[0].col, s.boundary_polygon[0].row)])
            ax.plot(sx, sy, color=colors[i % len(colors)], linestyle='--', linewidth=2, label=f'Stage {s.stage_id}')

    if path:
        px = [p.col for p in path]
        py = [p.row for p in path]

        ax2.plot(px, py, 'gray', linewidth=1, alpha=0.5, zorder=1, label='Path')

        stage_indices = [i for i, p in enumerate(path) if p.path_type.startswith('stage_')]
        connection_indices = [i for i, p in enumerate(path) if p.path_type == 'connection']

        if stage_indices:
            n_stage = len(stage_indices)
            colors_gradient = plt.cm.coolwarm(np.linspace(0, 1, n_stage))
            display_step = max(1, n_stage // 200)
            for idx, i in enumerate(stage_indices[::display_step]):
                color_idx = min(int(idx * len(stage_indices) / (len(stage_indices[::display_step]))), n_stage - 1)
                ax2.scatter(px[i], py[i], c=[colors_gradient[color_idx]],
                            s=25, zorder=3, edgecolors='none', alpha=0.7)

        if connection_indices:
            conn_segments = []
            current_segment = [connection_indices[0]]
            for i in range(1, len(connection_indices)):
                if connection_indices[i] == connection_indices[i - 1] + 1:
                    current_segment.append(connection_indices[i])
                else:
                    conn_segments.append(current_segment)
                    current_segment = [connection_indices[i]]
            conn_segments.append(current_segment)

            for segment in conn_segments:
                seg_x = [px[i] for i in segment]
                seg_y = [py[i] for i in segment]
                ax2.plot(seg_x, seg_y, color='purple', linewidth=3,
                         linestyle='--', alpha=0.9, zorder=2, label='Connection' if segment == conn_segments[0] else '')

        stage_markers = {}
        for i, point in enumerate(path):
            if point.path_type.startswith('stage_'):
                stage_id = point.path_type
                if stage_id not in stage_markers:
                    stage_markers[stage_id] = {'start_idx': i, 'end_idx': i}
                else:
                    stage_markers[stage_id]['end_idx'] = i

        stage_colors = {
            'stage_1': ('lime', 'darkgreen', 'blue'),
            'stage_2': ('cyan', 'darkblue', 'orange'),
            'stage_3': ('yellow', 'darkorange', 'red'),
            'stage_4': ('magenta', 'darkmagenta', 'purple')
        }

        for stage_id, indices in stage_markers.items():
            start_idx = indices['start_idx']
            end_idx = indices['end_idx']
            stage_num = stage_id.split('_')[1]

            if stage_id in stage_colors:
                start_color, start_edge, label_color = stage_colors[stage_id]
            else:
                start_color, start_edge, label_color = ('white', 'black', 'gray')

            ax2.scatter([px[start_idx]], [py[start_idx]],
                        c=start_color, s=80, marker='o',
                        zorder=9, edgecolors=start_edge, linewidths=2,
                        label=f'Stage {stage_num} Start')
            ax2.text(px[start_idx] - 1.5, py[start_idx], f'S{stage_num}',
                     fontsize=8, fontweight='bold',
                     ha='right', va='center', color=start_edge,
                     bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                               alpha=0.95, edgecolor=start_edge, linewidth=1.2))

            ax2.scatter([px[end_idx]], [py[end_idx]],
                        c='red', s=80, marker='s',
                        zorder=9, edgecolors='darkred', linewidths=2,
                        label=f'Stage {stage_num} End')
            ax2.text(px[end_idx] + 1.5, py[end_idx], f'E{stage_num}',
                     fontsize=8, fontweight='bold',
                     ha='left', va='center', color='darkred',
                     bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                               alpha=0.95, edgecolor='darkred', linewidth=1.2))

        if stage_indices:
            sm = plt.cm.ScalarMappable(cmap='coolwarm',
                                       norm=plt.Normalize(vmin=0, vmax=len(stage_indices)))
            sm.set_array([])
            cbar = plt.colorbar(sm, ax=ax2, orientation='vertical', pad=0.02, shrink=0.8)
            cbar.set_label('Path Progress\n(Blue→Red)', fontsize=11, fontweight='bold')

        info_text = f"Total Points: {len(path)}\n"
        info_text += f"Stages: {len(stage_markers)}\n"
        for stage_id, indices in sorted(stage_markers.items()):
            stage_num = stage_id.split('_')[1]
            stage_points = indices['end_idx'] - indices['start_idx'] + 1
            info_text += f"  Stage {stage_num}: {stage_points} pts\n"
        info_text += "\n✅ FIXED v3.2:\n"
        info_text += "  • Boundary check (radius=0.0)\n"
        info_text += "  • Post-filtering enabled\n"
        info_text += "  • Param logging added\n"
        info_text += "  • Inflation: 0.65m default"

        ax2.text(0.02, 0.98, info_text, transform=ax2.transAxes,
                 fontsize=9, verticalalignment='top', fontweight='bold',
                 bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.85))

    ax1.legend(loc='upper right', fontsize=9)
    ax2.legend(loc='lower right', fontsize=8, ncol=2)
    ax1.grid(True, alpha=0.3)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    # 处理输出路径
    import os

    if output_dir:
        if not os.path.exists(output_dir):
            try:
                os.makedirs(output_dir, exist_ok=True)
                print(f"   [创建目录] {output_dir}")
            except:
                print(f"   [警告] 无法创建目录 {output_dir}，使用当前目录")
                output_dir = None

        if output_dir:
            output_path = os.path.join(output_dir, filename)
        else:
            output_path = filename
    else:
        output_path = filename

    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✅ 路径规划结果已保存到: {output_path}")
    plt.close()


# ============================================================
# 演示函数
# ============================================================

def demo_fixed():
    """

    """
    print("=" * 70)
    print("参数打印")
    print("=" * 70)

    grid = np.zeros((100, 100), dtype=int)

    # 障碍物参数
    obstacle1 = [
        Point(row=20, col=15),
        Point(row=15, col=15),
        Point(row=15, col=25),
        Point(row=20, col=25)
    ]

    robot_config = RobotConfig(
        width=1.3,
        length=2.0,
        path_spacing=3.0,
        turning_radius=1.0,
        overlap_ratio=0.15,
        inflation_radius=0.65  # 使用安全的膨胀半径
    )

    # 全局区域
    global_area = [
        Point(row=10, col=10),
        Point(row=10, col=70),
        Point(row=40, col=70),
        Point(row=40, col=50),
        Point(row=20, col=50),
        Point(row=20, col=30),
        Point(row=40, col=30),
        Point(row=40, col=10)
    ]

    # 阶段1区域
    stage1_poly = [
        Point(row=10, col=10),
        Point(row=10, col=30),
        Point(row=40, col=30),
        Point(row=40, col=10)
    ]

    # 阶段2区域
    stage2_poly = [
        Point(row=10, col=30),
        Point(row=10, col=50),
        Point(row=20, col=50),
        Point(row=20, col=30)
    ]

    # 阶段3区域
    stage3_poly = [
        Point(row=10, col=50),
        Point(row=10, col=70),
        Point(row=40, col=70),
        Point(row=40, col=50)
    ]

    # 配置3个阶段
    stages = [
        StageConfig(
            stage_id=1,
            boundary_polygon=stage1_poly,
            start_point=Point(row=40, col=10),
            end_point=Point(row=15, col=30),
            direction='x',
            path_spacing=2.0,
            obstacle_polygons=[obstacle1]
        ),
        StageConfig(
            stage_id=2,
            boundary_polygon=stage2_poly,
            start_point=Point(row=10, col=35),
            end_point=Point(row=20, col=50),
            direction='y',
            path_spacing=2.0,
            obstacle_polygons=None
        ),
        StageConfig(
            stage_id=3,
            boundary_polygon=stage3_poly,
            start_point=Point(row=10, col=50),
            end_point=Point(row=40, col=70),
            direction='x',
            path_spacing=2.0,
            obstacle_polygons=None
        )
    ]

    print("\n说明：修复特性")
    print("   边界溢出修复 (radius=0.0)")
    print("   安全膨胀半径 (0.65m)")

    planner = MultiStagePathPlanner(robot_config)
    path = planner.plan(grid, global_area, stages, 1.0)

    if path:
        print(f"\n{'=' * 70}")
        print(f"验证结果")
        print(f"{'=' * 70}")

        for stage_id in [1, 2, 3]:
            stage_points = [p for p in path if p.path_type == f'stage_{stage_id}']
            if stage_points:
                print(f"\n阶段{stage_id}:")
                print(f"  起点: (x={stage_points[0].col:.1f}, y={stage_points[0].row:.1f})")
                print(f"  终点: (x={stage_points[-1].col:.1f}, y={stage_points[-1].row:.1f})")
                print(f"  点数: {len(stage_points)}")

        print(f"{'=' * 70}\n")

    import os
    output_dir = os.getcwd()

    export_path_to_json(path, '路径点输出.json', output_dir=output_dir)
    visualize_path(grid, global_area, stages, path, robot_config, output_dir=output_dir, filename='路径规划.png')

    return path


# ============================================================
# 主程序入口
# ============================================================

if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════════════════╗
║ 四局你好！                     
╚══════════════════════════════════════════════════════════════════════╝
    """)

    path = demo_fixed()

    print("\n" + "=" * 70)
    print("✅ 演示完成!")
    print("=" * 70)
    print("\n输出文件:")
    print("  1. 路径点输出.json")
    print("  2. 路径规划.png")
    print("\n" + "=" * 70)