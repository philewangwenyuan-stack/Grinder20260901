#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
修复版 v4.7.1
"""

import numpy as np
import matplotlib

matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
import heapq
import math
import json
import os
import sys
import time
from collections import deque
from dataclasses import dataclass, asdict
from typing import List, Optional, Tuple
from matplotlib.path import Path as MplPath
from scipy import ndimage
from datetime import datetime

# PlannerAdapter loads this file dynamically. Add its directory so a locally
# built extension can be found before falling back to the Python implementation.
_planner_dir = os.path.dirname(os.path.abspath(__file__))
if _planner_dir not in sys.path:
    sys.path.insert(0, _planner_dir)

try:
    import _mst27_cpp
    _mst27_cpp_import_error = ""
except ImportError as exc:
    _mst27_cpp = None
    _mst27_cpp_import_error = repr(exc)


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
    endpoint_margin: Optional[float] = None  # 米；None时使用RobotConfig.endpoint_margin


@dataclass
class RobotConfig:
    width: float = 1.2  # 机器人作业宽度，单位：米
    length: float = 1.85  # 机器人车身长度，单位：米
    path_spacing: float = 0.7  # 相邻基准扫掠线的中心间距，单位：米
    turning_radius: float = 0.1  # 最小转弯半径，单位：米
    overlap_ratio: float = 0.0  # 扫掠线重叠比例；0表示直接使用path_spacing
    inflation_radius: float = 0.6  # 原始障碍物安全膨胀距离，单位：米
    endpoint_margin: float = 2.0  # 扫掠线起止点距作业边界的留边，单位：米
    output_point_spacing: float = 0.3  # 最终输出路径的补点间距，单位：米；0表示关闭补点
    aligned_obstacle_inflation: float = 0.7  # 角度对齐后的柱子外接矩形膨胀距离，单位：米
    aligned_obstacle_max_extent: float = 3.0  # 柱子识别的最大边长，单位：米；超过后按墙体处理
    obstacle_corner_angle_deg: float = 45.0  # 绕柱外推四个锚点的进入和退出角度，单位：度
    obstacle_avoidance_distance: float = 0.1  # 在0.6米柱子膨胀范围外追加的避让距离，单位：米


def _direction_axis(direction):
    """返回去掉正负号的扫掠轴；非法值按 x 处理。"""
    axis = str(direction or "x").strip().lower().lstrip("+-")
    return axis if axis in ("x", "y") else "x"


def _direction_travel_sign(direction):
    """首条作业线沿旋转后轴向的行进符号。"""
    return -1 if str(direction or "x").strip().startswith("-") else 1


# ============================================================
# 障碍物处理器
# ============================================================

class ObstacleProcessor:
    @staticmethod
    def inflate_obstacles(grid_map: np.ndarray, inflation_radius_pixels: int) -> np.ndarray:
        if inflation_radius_pixels <= 0:
            return grid_map.copy()
        occupied = np.asarray(grid_map) != 0
        if not np.any(occupied):
            return np.zeros_like(grid_map, dtype=int)
        # 欧氏距离变换与圆盘形态学膨胀逐格等价，但不会随半径增大而构造
        # 巨型结构核；1.5m/5cm=30格时可显著降低预处理时间。
        distance = ndimage.distance_transform_edt(~occupied)
        inflated = (distance <= float(inflation_radius_pixels) + 1e-9).astype(int)

        print(f"   [障碍物膨胀] 膨胀半径: {inflation_radius_pixels}像素, "
              f"原始: {np.sum(grid_map)}, 膨胀后: {np.sum(inflated)}")

        return inflated

    @staticmethod
    def inflate_axis_aligned_obstacles(
            grid_map: np.ndarray, inflation_radius_pixels: int) -> np.ndarray:
        """用方形结构元素膨胀，保持水平/垂直边界，不产生圆角。"""
        if inflation_radius_pixels <= 0:
            return np.asarray(grid_map, dtype=int).copy()

        occupied = np.asarray(grid_map, dtype=bool)
        if not np.any(occupied):
            return np.zeros_like(occupied, dtype=int)

        radius = int(inflation_radius_pixels)
        structure = np.ones((2 * radius + 1, 2 * radius + 1), dtype=bool)
        inflated = ndimage.binary_dilation(occupied, structure=structure)

        print(f"   [矩形障碍物膨胀] 膨胀半径: {radius}像素, "
              f"原始: {np.sum(occupied)}, 膨胀后: {np.sum(inflated)}")
        return inflated.astype(int)

    @staticmethod
    def angle_aligned_rectangles(grid_map: np.ndarray, angle_deg: float) -> np.ndarray:
        """每个独立障碍连通域 → 与规划角度(d,n)对齐的保守外接矩形。"""
        occupied = np.asarray(grid_map, dtype=bool)
        result = np.zeros_like(occupied, dtype=np.uint8)
        labels, count = ndimage.label(
            occupied, structure=np.ones((3, 3), dtype=np.uint8))
        theta = math.radians(float(angle_deg))
        dx, dy = math.cos(theta), math.sin(theta)
        nx, ny = -dy, dx
        # 一个栅格单元投影到旋转轴后的半宽，确保矩形完整包含原障碍栅格。
        half_d = 0.5 * (abs(dx) + abs(dy))
        half_n = 0.5 * (abs(nx) + abs(ny))

        objects = ndimage.find_objects(labels)
        for label_id, obj in enumerate(objects, start=1):
            if obj is None:
                continue
            local = labels[obj] == label_id
            rr0, cc0 = np.nonzero(local)
            if rr0.size == 0:
                continue
            rr = rr0 + obj[0].start
            cc = cc0 + obj[1].start
            tt = cc * dx + rr * dy
            ss = cc * nx + rr * ny
            t_min, t_max = float(tt.min() - half_d), float(tt.max() + half_d)
            s_min, s_max = float(ss.min() - half_n), float(ss.max() + half_n)

            corners = [
                (t_min * dx + s_min * nx, t_min * dy + s_min * ny),
                (t_min * dx + s_max * nx, t_min * dy + s_max * ny),
                (t_max * dx + s_min * nx, t_max * dy + s_min * ny),
                (t_max * dx + s_max * nx, t_max * dy + s_max * ny),
            ]
            c_lo = max(0, int(math.floor(min(x for x, _ in corners))) - 1)
            c_hi = min(result.shape[1] - 1,
                       int(math.ceil(max(x for x, _ in corners))) + 1)
            r_lo = max(0, int(math.floor(min(y for _, y in corners))) - 1)
            r_hi = min(result.shape[0] - 1,
                       int(math.ceil(max(y for _, y in corners))) + 1)
            if r_lo > r_hi or c_lo > c_hi:
                continue
            c_grid, r_grid = np.meshgrid(
                np.arange(c_lo, c_hi + 1), np.arange(r_lo, r_hi + 1))
            t_grid = c_grid * dx + r_grid * dy
            s_grid = c_grid * nx + r_grid * ny
            inside = ((t_grid >= t_min - 1e-9) & (t_grid <= t_max + 1e-9) &
                      (s_grid >= s_min - 1e-9) & (s_grid <= s_max + 1e-9))
            result[r_lo:r_hi + 1, c_lo:c_hi + 1] |= inside.astype(np.uint8)

        print(f"   [角度外接矩形] angle={float(angle_deg):.4f}°, "
              f"连通障碍={count}, 原始={int(occupied.sum())}格, "
              f"矩形化={int(result.sum())}格")
        return result

    @staticmethod
    def build_angle_aligned_inflated(
            grid_map: np.ndarray, angle_deg: float, inflation_pixels: int,
            base_inflation_pixels: int = 0,
            max_component_extent_pixels: Optional[int] = None) -> np.ndarray:
        """
        仅将独立小障碍（柱子）转换为规划角度对齐矩形。

        与图像边缘连通的外围/墙体，以及尺寸超过柱子上限的长大连通域，
        保留原始轮廓并只做机器人基础安全半径膨胀，避免其旋转外接矩形
        横跨工作区。
        """
        occupied = np.asarray(grid_map, dtype=bool)
        if not np.any(occupied):
            return np.zeros_like(occupied, dtype=int)

        labels, count = ndimage.label(
            occupied, structure=np.ones((3, 3), dtype=np.uint8))
        preserve = np.zeros_like(occupied, dtype=bool)
        column_obstacles = np.zeros_like(occupied, dtype=bool)
        objects = ndimage.find_objects(labels)
        border_ids = set(np.unique(np.concatenate((
            labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1]))))
        border_ids.discard(0)
        extent_limit = (None if max_component_extent_pixels is None else
                        max(1, int(max_component_extent_pixels)))
        border_count = 0
        large_count = 0
        column_count = 0

        for label_id, obj in enumerate(objects, start=1):
            if obj is None:
                continue
            local = labels[obj] == label_id
            height = int(obj[0].stop - obj[0].start)
            width = int(obj[1].stop - obj[1].start)
            touches_border = label_id in border_ids
            too_large = (extent_limit is not None and
                         max(height, width) > extent_limit)
            target = preserve if (touches_border or too_large) else column_obstacles
            target[obj] |= local
            if touches_border:
                border_count += 1
            elif too_large:
                large_count += 1
            else:
                column_count += 1

        rectangles = ObstacleProcessor.angle_aligned_rectangles(
            column_obstacles, angle_deg)
        # 柱子安全区使用方形结构元素膨胀，避免矩形四角被欧氏距离
        # 膨胀成圆角；基础机器人安全半径仍使用上面的欧氏膨胀。
        columns_inflated = ObstacleProcessor.inflate_axis_aligned_obstacles(
            rectangles, max(0, int(inflation_pixels)))
        preserved_inflated = ObstacleProcessor.inflate_obstacles(
            preserve.astype(np.uint8), max(0, int(base_inflation_pixels)))
        result = np.logical_or(columns_inflated, preserved_inflated).astype(int)
        print(f"   [障碍分类] 连通域={count}, 柱子={column_count}, "
              f"贴边外围/墙体={border_count}, 超尺寸墙体={large_count}, "
              f"柱子上限={extent_limit if extent_limit is not None else '不限'}像素")
        return result

    @staticmethod
    def polygon_to_grid(polygon_points: List[Point], grid_shape: Tuple[int, int]) -> np.ndarray:
        """
        将多边形障碍物转换为grid格式
        """
        return ObstacleProcessor.polygon_to_mask(polygon_points, grid_shape).astype(int)

    @staticmethod
    def polygon_to_mask(polygon_points: List[Point], grid_shape: Tuple[int, int],
                        radius: float = 0.0) -> np.ndarray:
        mask = np.zeros(grid_shape, dtype=bool)
        if not polygon_points:
            return mask

        poly_verts = [(p.col, p.row) for p in polygon_points]
        ny, nx = grid_shape

        rows = [p.row for p in polygon_points]
        cols = [p.col for p in polygon_points]
        min_row = max(0, int(math.floor(min(rows))))
        max_row = min(ny - 1, int(math.ceil(max(rows))))
        min_col = max(0, int(math.floor(min(cols))))
        max_col = min(nx - 1, int(math.ceil(max(cols))))
        if min_row > max_row or min_col > max_col:
            return mask

        x, y = np.meshgrid(np.arange(min_col, max_col + 1),
                           np.arange(min_row, max_row + 1))
        points = np.column_stack((x.ravel(), y.ravel()))
        path = MplPath(poly_verts)
        local_mask = path.contains_points(points, radius=radius).reshape(
            (max_row - min_row + 1, max_col - min_col + 1)
        )
        mask[min_row:max_row + 1, min_col:max_col + 1] = local_mask

        return mask


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
        grid = ObstacleProcessor.polygon_to_mask(polygon_points, mask.shape)
        mask[grid] = True


# ============================================================
# 核心弓字形规划器
# ============================================================

class ObstacleAwareBoustrophedonPlanner:
    def __init__(self, robot_config: RobotConfig):
        self.robot_config = robot_config
        self._polygon_path_cache = {}

    def _get_polygon_path(self, polygon: List[Point]) -> MplPath:
        key = tuple((p.col, p.row) for p in polygon)
        path = self._polygon_path_cache.get(key)
        if path is None:
            path = MplPath(key)
            self._polygon_path_cache[key] = path
        return path

    def _is_point_in_polygon(self, point: Point, polygon: List[Point], on_boundary_counts=True) -> bool:
        """
        判断点是否在多边形内部

        Args:
            point: 待判断的点
            polygon: 多边形顶点列表
            on_boundary_counts: 边界上的点是否算作"在内部" (默认True)
        """
        path = self._get_polygon_path(polygon)

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

        queue = deque([(p.row, p.col)])
        visited = set([(p.row, p.col)])
        max_search = 1000
        count = 0

        while queue and count < max_search:
            r, c = queue.popleft()
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

    def _route_astar(self, grid_map, a, b):
        """A* 搜索阶段直接返回方向变化点；不连通时返回 None。"""
        pa = Point(int(round(a.row)), int(round(a.col)))
        pb = Point(int(round(b.row)), int(round(b.col)))
        if pa.row == pb.row and pa.col == pb.col:
            return []
        route = self._astar_search(grid_map, pa, pb)
        if not route or len(route) < 2:
            return None
        if route[-1].row != pb.row or route[-1].col != pb.col:
            return None

        # 仅对A*角点做可视直连（string pulling），消除整数栅格产生的
        # 一格锯齿；不处理整条覆盖路径，开销只与少量A*角点有关。
        rows, cols = grid_map.shape
        def _visible(u, v):
            dist = math.hypot(v.col - u.col, v.row - u.row)
            count = max(1, int(math.ceil(dist / 0.25)))
            for k in range(count + 1):
                t = k / count
                r = int(round(u.row + (v.row - u.row) * t))
                c = int(round(u.col + (v.col - u.col) * t))
                if not (0 <= r < rows and 0 <= c < cols) or grid_map[r, c] != 0:
                    return False
            return True

        pulled = [route[0]]
        anchor = 0
        while anchor < len(route) - 1:
            farthest = anchor + 1
            probe = farthest + 1
            while probe < len(route) and _visible(route[anchor], route[probe]):
                farthest = probe
                probe += 1
            pulled.append(route[farthest])
            anchor = farthest
        return pulled[1:-1]

    def plan_subregion_oriented(self, grid_map, boundary_polygon, angle_deg,
                                path_spacing, step=0.5,
                                start_point=None, end_point=None,
                                endpoint_margin=0.0,
                                first_travel_sign=None):
        """
        任意角度 / 任意精度弓字形规划（不旋转栓格）。
        栓格仅作障碍查询表，扫描线在连续坐标下沿方向向量切分。

        Args:
            grid_map: 障碍栓格 (0=空闲, 1=障碍)，建议传膨胀后的 inflated_grid
            boundary_polygon: 子区域边界多边形 (List[Point])
            angle_deg: 扫描前进方向与 col(+x) 轴夹角，单位度，支持任意浮点(如 37.1234)
            path_spacing: 相邻扫描线间距（格）
            step: 沿扫描线采样步长（格），可 <1 取亚格精度
            start_point / end_point: 可选起终点
            first_travel_sign: 首条作业线方向，+1 沿旋转后的正轴，-1 沿负轴；
                               None 保留按起点就近选择的旧行为
        Returns:
            List[PathPoint]（row/col 为浮点）
        """
        if not boundary_polygon:
            return []

        # RK3588 稀疏角点模式内部按 1 格采样。5cm 栅格下仍逐格检查障碍，
        # 但不再为最终会被删除的半格中间点付出双倍计算量。
        step = max(float(step), 1.0)

        theta = math.radians(float(angle_deg))
        dx, dy = math.cos(theta), math.sin(theta)      # 前进方向 (x=col, y=row)
        nx, ny = -math.sin(theta), math.cos(theta)     # 扫描线推进法向

        H, W = grid_map.shape
        verts = [(float(p.col), float(p.row)) for p in boundary_polygon]
        proj_d = [x * dx + y * dy for x, y in verts]
        proj_n = [x * nx + y * ny for x, y in verts]
        d_min, d_max = min(proj_d), max(proj_d)
        n_min, n_max = min(proj_n), max(proj_n)
        poly = MplPath(verts)

        def inside_region(x, y):
            """只判断连续点是否位于规划多边形内，不把边界当障碍绕行。"""
            c, r = int(round(x)), int(round(y))
            if not (0 <= r < H and 0 <= c < W):
                return False
            return poly.contains_point((x, y), radius=-0.5)

        def is_free(x, y):
            c, r = int(round(x)), int(round(y))
            if not inside_region(x, y):
                return False
            if grid_map[r, c] != 0:
                return False
            return True

        def _append_corner(dst, p, eps=1e-7):
            """规划时在线压缩共线点，只保存线段端点/真实角点，O(1)/点。"""
            if dst:
                dc = p.col - dst[-1].col
                dr = p.row - dst[-1].row
                if dc * dc + dr * dr <= eps * eps:
                    dst[-1] = p
                    return
            if len(dst) >= 2:
                a, b = dst[-2], dst[-1]
                v1c, v1r = b.col - a.col, b.row - a.row
                v2c, v2r = p.col - b.col, p.row - b.row
                cross = v1c * v2r - v1r * v2c
                scale = max(1.0, math.hypot(v1c, v1r) * math.hypot(v2c, v2r))
                # 同向且共线：延长当前线段，而不是先生成密集点再做 RDP。
                if abs(cross) <= eps * scale and v1c * v2c + v1r * v2r >= 0.0:
                    dst[-1] = p
                    return
            dst.append(p)

        def _extend_corners(dst, points):
            for point in points:
                _append_corner(dst, point)

        all_segments = []
        segment_line_ids = []
        segment_base_s = []

        def _append_oriented_segment(seg, line_id, base_s):
            """保存扫描段及其真实来源；后续连接禁止根据偏移端点猜 line_id。"""
            if not seg:
                return
            all_segments.append(seg)
            segment_line_ids.append(int(line_id))
            segment_base_s.append(float(base_s))

        # 扫描范围必须覆盖区域完整法向宽度 [n_min, n_max]。
        # 起终点的法向投影只用于选择从哪一侧开始，不能作为扫描范围端点；
        # 否则在0°附近且起终点近似同高时会退化成仅一条扫描线。
        def _clamp(v, lo, hi):
            return max(lo, min(hi, v))

        start_s_hint = (_clamp(start_point.col * nx + start_point.row * ny, n_min, n_max)
                        if start_point is not None else n_min)
        end_s_hint = (_clamp(end_point.col * nx + end_point.row * ny, n_min, n_max)
                      if end_point is not None else n_max)
        # 现场指定起点优先决定扫掠侧序：从离起点最近的法向边缘开始，
        # 向对侧覆盖。终点不允许反过来改变已经选定的起始侧。
        if abs(start_s_hint - n_min) <= abs(start_s_hint - n_max):
            start_edge, end_edge = n_min, n_max
        else:
            start_edge, end_edge = n_max, n_min
        s_dir = 1.0 if end_edge >= start_edge else -1.0
        normal_inset = min(0.5 * float(path_spacing), 0.5 * (n_max - n_min))
        # 显式现场选点就是首/末作业线的法向坐标；首末间隔允许自适应，
        # 中间线仍保持 path_spacing。没有显式点时才使用半间距内缩线。
        s_start = (start_s_hint if start_point is not None
                   else start_edge + s_dir * normal_inset)
        s_end = (end_s_hint if end_point is not None
                 else end_edge - s_dir * normal_inset)
        start_t = (start_point.col * dx + start_point.row * dy
                   if start_point is not None else d_min)
        end_t = (end_point.col * dx + end_point.row * dy
                 if end_point is not None else d_max)
        natural_first_ascending = abs(start_t - d_min) <= abs(start_t - d_max)
        forced_first_ascending = None
        if first_travel_sign is not None:
            forced_first_ascending = float(first_travel_sign) >= 0.0
        first_ascending = (natural_first_ascending if forced_first_ascending is None
                           else forced_first_ascending)
        # 只有当现场起点沿指定方向已无可用作业段时才转场。
        # 若只是“更靠近右边”但前方仍有空间，重规划应从当前点继续 +x，
        # 不应因为就近策略先返回左端。
        reposition_to_forced_start = False
        wanted_last_ascending = abs(end_t - d_max) <= abs(end_t - d_min)

        # 唯一规划范围就是现场起点线到现场终点线。禁止为了“全区域覆盖”
        # 去起点另一侧、越过终点或完成后再折返。
        span = abs(s_end - s_start)
        if span <= 1e-9:
            if forced_first_ascending is None:
                first_ascending = end_t >= start_t
            wanted_last_ascending = first_ascending
            scan_s = [s_start]
        else:
            intervals = max(1, int(round(span / path_spacing)))
            need_even = (first_ascending == wanted_last_ascending)
            candidates = [n for n in range(max(1, intervals - 2), intervals + 3)
                          if (n % 2 == 0) == need_even]
            intervals = min(candidates,
                            key=lambda n: abs(span / n - path_spacing))
            scan_s = [s_start + (s_end - s_start) * i / intervals
                      for i in range(intervals + 1)]

        # 不再为匹配终点方向增删扫描线；末线在生成时直接采用终点方向。

        n_lines = 0
        max_off = path_spacing * 2.5          # 单侧横移上限(格)
        o_step = 1.0                          # 横移搜索粒度(格)
        # 几何约束分离：扫描线掉头保持90°；柱子侧让角由调参项控制。
        corner_angle_deg = float(self.robot_config.obstacle_corner_angle_deg)
        corner_tangent = math.tan(math.radians(corner_angle_deg))
        slope = corner_tangent * step
        effective_spacing_m = max(
            1e-9,
            self.robot_config.path_spacing *
            (1.0 - self.robot_config.overlap_ratio))
        pixels_per_meter = float(path_spacing) / effective_spacing_m
        avoidance_pixels = max(
            0.0,
            float(self.robot_config.obstacle_avoidance_distance) *
            pixels_per_meter)

        def _req(ts_line, s_line, sgn):
            """本条扫描线每个采样点在 sgn 侧所需的最小横移量(格)；无法清空则 None。"""
            req = []
            for tt in ts_line:
                bx = tt * dx + s_line * nx
                by = tt * dy + s_line * ny
                if is_free(bx, by):
                    req.append(0.0)
                    continue
                o = o_step
                hit = None
                while o <= max_off:
                    if is_free(bx + sgn * o * nx, by + sgn * o * ny):
                        hit = o
                        break
                    o += o_step
                req.append(hit)
            return req

        for k, s in enumerate(scan_s):
            n_lines += 1
            ascending = first_ascending if k % 2 == 0 else not first_ascending
            # 先生成整条边界截线，留边后再应用显式起终点裁剪。
            # 这样X/Y/任意角度都沿实际扫描方向d留出相同距离。
            t_lo, t_hi = d_min, d_max

            ts = []
            t = t_lo
            while t < t_hi - 1e-9:
                ts.append(t)
                t += step
            ts.append(t_hi)
            # 关键修复：先按真实边界裁剪扫描线。边界外不是柱子，不能进入
            # 单侧侧让���算，否则会在斜边处产生锯齿/尖����和长 A* 回接。
            ts = [tt for tt in ts
                  if inside_region(tt * dx + s * nx, tt * dy + s * ny)]
            margin = max(0.0, float(endpoint_margin))
            if ts and margin > 1e-9:
                edge_lo, edge_hi = min(ts), max(ts)
                inner_lo, inner_hi = edge_lo + margin, edge_hi - margin
                if inner_lo >= inner_hi - 1e-9:
                    continue
                middle = [tt for tt in ts if inner_lo < tt < inner_hi]
                ts = [inner_lo, *middle, inner_hi]

            if (k == 0 and start_point is not None and ts and
                    forced_first_ascending is not None):
                forward_span = ((max(ts) - start_t) if ascending
                                else (start_t - min(ts)))
                reposition_to_forced_start = forward_span <= 0.5 * max(step, 1.0)

            # 显式起终点只负责进一步缩短首末扫描线，不会突破留边范围。
            if (k == 0 and start_point is not None and
                    not reposition_to_forced_start):
                if ascending:
                    ts = [tt for tt in ts if tt >= start_t - 1e-9]
                else:
                    ts = [tt for tt in ts if tt <= start_t + 1e-9]
            if k == len(scan_s) - 1 and end_point is not None:
                if ascending:
                    ts = [tt for tt in ts if tt <= end_t + 1e-9]
                else:
                    ts = [tt for tt in ts if tt >= end_t - 1e-9]
            # 连续多边形内的端点，四舍五入后可能落到“边界外障碍格”。
            # 这类点必须沿扫掠方向向内裁剪，不能当成柱子做法向侧移，
            # 否则近0°斜边会出现周期性三角毛刺。
            def _base_free(tt):
                return is_free(tt * dx + s * nx, tt * dy + s * ny)
            while ts and not _base_free(ts[0]):
                ts.pop(0)
            while ts and not _base_free(ts[-1]):
                ts.pop()
            if not ts:
                continue
            if not ascending:
                ts.reverse()

            # 两侧各算一次，自动选“更空”的一侧，并在整条线上保持一致
            req_p = _req(ts, s, +1.0)
            req_m = _req(ts, s, -1.0)
            none_p = sum(1 for r in req_p if r is None)
            none_m = sum(1 for r in req_m if r is None)
            sum_p = sum(r for r in req_p if r)
            sum_m = sum(r for r in req_m if r)
            if (none_p, sum_p) <= (none_m, sum_m):
                sgn, req = 1.0, req_p
            else:
                sgn, req = -1.0, req_m

            cur = []
            raw_last = None
            raw_dir = None

            def _emit_raw_corner(py, px):
                """只在方向改变时构造 PathPoint；直线中间采样不创建对象。"""
                nonlocal raw_last, raw_dir, cur
                current = (float(py), float(px))
                if raw_last is None:
                    cur = [PathPoint(current[0], current[1])]
                    raw_last = current
                    raw_dir = None
                    return
                vr = current[0] - raw_last[0]
                vc = current[1] - raw_last[1]
                if raw_dir is None:
                    raw_dir = (vr, vc)
                else:
                    cross = raw_dir[1] * vr - raw_dir[0] * vc
                    scale = max(1.0, math.hypot(*raw_dir) * math.hypot(vr, vc))
                    if (abs(cross) > 1e-7 * scale or
                            raw_dir[0] * vr + raw_dir[1] * vc < 0.0):
                        cur.append(PathPoint(raw_last[0], raw_last[1]))
                        raw_dir = (vr, vc)
                raw_last = current

            def _flush_raw_segment():
                """提交当前线段终点；每条直线最终只有首尾两个点。"""
                nonlocal raw_last, raw_dir, cur
                if raw_last is not None:
                    if (not cur or abs(cur[-1].row - raw_last[0]) > 1e-9 or
                            abs(cur[-1].col - raw_last[1]) > 1e-9):
                        cur.append(PathPoint(raw_last[0], raw_last[1]))
                if cur:
                    _append_oriented_segment(cur, k, s)
                cur = []
                raw_last = None
                raw_dir = None

            n = len(ts)
            i = 0
            while i < n:
                if req[i] is None:
                    # 两侧都无法清空(如柱子贴边界) → 断开，交给 A* 兜底
                    _flush_raw_segment()
                    i += 1
                    continue
                j = i
                while j < n and req[j] is not None:
                    j += 1
                # 不逐点跟随柱子栅格轮廓。每个非零需求区间只取一次最大
                # 侧移量，并按配置折角构造梯形；掉头仍由后续逻辑保持90°。
                raw = [req[x] for x in range(i, j)]
                r = [0.0] * len(raw)
                q = 0
                while q < len(raw):
                    if raw[q] <= 1e-9:
                        q += 1
                        continue
                    lo = q
                    # 同一柱子的量化轮廓可能夹有1~2个零点，合并成一个梯形。
                    last_hit = q
                    while q + 1 < len(raw):
                        q += 1
                        if raw[q] > 1e-9:
                            last_hit = q
                        elif q - last_hit > 2:
                            break
                    hi = last_hit
                    peak = max(raw[lo:hi + 1]) + avoidance_pixels
                    ramp_n = max(1, int(math.ceil(peak / max(slope, 1e-9))))
                    # 端部附近优先预留完整配置角度回正坡；只有剩余距离充足时
                    # 才扩展最多4格平台，禁止平台把回正坡挤出扫描线端点。
                    left_pad = 0
                    right_pad = 0
                    plateau_lo = lo - left_pad
                    plateau_hi = hi + right_pad
                    left = max(0, plateau_lo - ramp_n)
                    right = min(len(raw) - 1, plateau_hi + ramp_n)
                    for x in range(left, plateau_lo):
                        r[x] = max(r[x], max(0.0, peak - (plateau_lo - x) * slope))
                    for x in range(plateau_lo, plateau_hi + 1):
                        r[x] = max(r[x], peak)
                    for x in range(plateau_hi + 1, right + 1):
                        r[x] = max(r[x], max(0.0, peak - (x - plateau_hi) * slope))
                    q = max(q, hi + 1)
                # 从两端施加配置折角回正上限；后续逐点碰撞检查会在必要处重新
                # 增大偏移，因此不会为了回正而切入柱子膨胀区。
                for x in range(len(r)):
                    endpoint_envelope = min(x, len(r) - 1 - x) * slope
                    r[x] = min(r[x], endpoint_envelope)
                for xi in range(i, j):
                    tt = ts[xi]
                    bx = tt * dx + s * nx
                    by = tt * dy + s * ny
                    oo = r[xi - i]
                    ok = is_free(bx + sgn * oo * nx, by + sgn * oo * ny)
                    guard = 0
                    while not ok and oo <= max_off and guard < 400:
                        oo += o_step
                        ok = is_free(bx + sgn * oo * nx, by + sgn * oo * ny)
                        guard += 1
                    if not ok:
                        _flush_raw_segment()
                        continue
                    px = bx + sgn * oo * nx
                    py = by + sgn * oo * ny
                    # 斜坡每步沿d移动step、沿n移动tan(angle)*step。
                    if raw_last is not None and (
                            abs(py - raw_last[0]) + abs(px - raw_last[1])) > (
                                1.1 * (1.0 + corner_tangent) *
                                max(step, 1.0)):
                        _flush_raw_segment()
                    _emit_raw_corner(py, px)
                i = j
            _flush_raw_segment()
        print(f"   [任意角度扫描] angle={float(angle_deg):.4f}°, 扫描线 {n_lines} 条, "
              f"段数 {len(all_segments)}, step={step}, 中间间距={path_spacing}")

        # 现场选点必须成为首/末作业线的真实端点，而不是由A*事后“经过”。
        # 只检查选点与已生成线端之间的开区间；选点本身可能恰在多边形边界。
        def _strict_endpoint_leg_free(a, b, exclude_b=False):
            dist = math.hypot(b.col - a.col, b.row - a.row)
            count = max(1, int(math.ceil(dist / 0.5)))
            last_q = count - 1 if exclude_b else count
            for q in range(1, last_q + 1):
                u = q / count
                if not is_free(a.col + (b.col - a.col) * u,
                               a.row + (b.row - a.row) * u):
                    return False
            return True

        if start_point is not None:
            if not all_segments or segment_line_ids[0] != 0:
                print("   [严格起点失败] 第一条作业线无有效段，拒绝移动现场起点")
                return []
            if not reposition_to_forced_start:
                exact_start = PathPoint(float(start_point.row), float(start_point.col))
                first_work = all_segments[0][0]
                if not _strict_endpoint_leg_free(exact_start, first_work):
                    print("   [严格起点失败] 现场起点到第一条作业线之间不可直达")
                    return []
                if math.hypot(first_work.row - exact_start.row,
                              first_work.col - exact_start.col) <= 1e-7:
                    all_segments[0][0] = exact_start
                else:
                    all_segments[0].insert(0, exact_start)
            else:
                print("   [定向重规划] 起点位于指定行进方向末端，先连接至首线起端")

        if end_point is not None:
            last_line_id = len(scan_s) - 1
            if not all_segments or segment_line_ids[-1] != last_line_id:
                print("   [严格终点失败] 最后一条作业线无有效段，拒绝移动现场终点")
                return []
            exact_end = PathPoint(float(end_point.row), float(end_point.col))
            last_work = all_segments[-1][-1]
            if not _strict_endpoint_leg_free(last_work, exact_end, exclude_b=True):
                print("   [严格终点失败] 最后一条作业线到现场终点之间不可直达")
                return []
            if math.hypot(last_work.row - exact_end.row,
                          last_work.col - exact_end.col) <= 1e-7:
                all_segments[-1][-1] = exact_end
            else:
                all_segments[-1].append(exact_end)

        # 诊断：检查每条扫描线首末点是否已回到原始 base_s。
        line_ends = {}
        for seg, lid, bs in zip(all_segments, segment_line_ids, segment_base_s):
            if lid not in line_ends:
                line_ends[lid] = [seg[0], seg[-1], bs]
            else:
                line_ends[lid][1] = seg[-1]
        bad_end_offsets = []
        for lid, (p_first, p_last, bs) in sorted(line_ends.items()):
            off_first = p_first.col * nx + p_first.row * ny - bs
            off_last = p_last.col * nx + p_last.row * ny - bs
            if abs(off_first) > 0.25 or abs(off_last) > 0.25:
                bad_end_offsets.append((lid, off_first, off_last))
        if bad_end_offsets:
            preview = ", ".join(
                f"L{lid}:({a:.1f},{b:.1f})" for lid, a, b in bad_end_offsets[:12])
            print(f"   [端点未回正] {len(bad_end_offsets)}条: {preview}")

        if not all_segments:
            return []

        # 不再把扫描段首末角点强行改成用户点；这种替换会把原线段拉成尖刺。
        # 起终点在后面通过角点化 A* 安全接入。

        def _bridge_line_free(a, b, sample_step=0.5):
            dist = math.hypot(b.col - a.col, b.row - a.row)
            count = max(1, int(math.ceil(dist / sample_step)))
            for q in range(1, count + 1):
                u = q / count
                if not is_free(a.col + (b.col - a.col) * u,
                               a.row + (b.row - a.row) * u):
                    return False
            return True

        def _append_path_corner(dst, p):
            """保留旋转坐标系中的90°角，禁止直连把工字形拉成斜线。"""
            _append_corner(dst, p)

        def _append_connection_corner(dst, p):
            """A*/跨段连接只走d/n轴或配置折角，禁止任意斜率。"""
            if dst:
                a = dst[-1]
                at = a.col * dx + a.row * dy
                ass = a.col * nx + a.row * ny
                bt = p.col * dx + p.row * dy
                bs = p.col * nx + p.row * ny
                if abs(bt - at) > 1e-6 and abs(bs - ass) > 1e-6:
                    shaped = False
                    candidates = [
                        PathPoint(bt * dy + ass * ny, bt * dx + ass * nx),
                        PathPoint(at * dy + bs * ny, at * dx + bs * nx),
                    ]
                    for corner in candidates:
                        if (_bridge_line_free(a, corner) and
                                _bridge_line_free(corner, p)):
                            _append_corner(dst, corner)
                            shaped = True
                            break
                    else:
                        # 纯直角狗腿受阻时，允许“配置折角 + 轴向直线”，
                        # 仍禁止输出任意斜率的A*弦线。
                        dt, ds = bt - at, bs - ass
                        m_t = min(abs(dt), abs(ds) / corner_tangent)
                        m_s = corner_tangent * m_t
                        st = 1.0 if dt >= 0.0 else -1.0
                        ss = 1.0 if ds >= 0.0 else -1.0
                        hybrid = [
                            PathPoint(
                                (at + st * m_t) * dy +
                                (ass + ss * m_s) * ny,
                                (at + st * m_t) * dx +
                                (ass + ss * m_s) * nx),
                            PathPoint(
                                (bt - st * m_t) * dy +
                                (bs - ss * m_s) * ny,
                                (bt - st * m_t) * dx +
                                (bs - ss * m_s) * nx),
                        ]
                        for corner in hybrid:
                            if (_bridge_line_free(a, corner) and
                                    _bridge_line_free(corner, p)):
                                _append_corner(dst, corner)
                                shaped = True
                                break
                    if not shaped:
                        return False
            _append_corner(dst, p)
            return True

        def _append_connection_with_backtrack(dst, p, min_index):
            """末端落入栅格对角死角时，回退当前作业线尾部再做受约束连接。"""
            if not dst:
                return False
            floor = max(0, min(int(min_index), len(dst) - 1))
            for tail in range(len(dst) - 1, floor - 1, -1):
                candidate = list(dst[:tail + 1])
                if _append_connection_corner(candidate, p):
                    dst[:] = candidate
                    return True
            return False

        def _extend_path_corners(dst, points):
            for point in points:
                _append_path_corner(dst, point)

        full_path = []
        # 起点始终作为第一个角点；距离较远时用只输出角点的 A* 接入。
        if start_point is not None:
            first = all_segments[0][0]
            sp0 = Point(int(round(start_point.row)), int(round(start_point.col)))
            _append_path_corner(full_path, PathPoint(float(sp0.row), float(sp0.col)))
            if abs(sp0.row - first.row) + abs(sp0.col - first.col) > 1.5:
                mid = self._route_astar(grid_map, sp0, first)
                if mid is None:
                    return []
                for p in mid:
                    _append_connection_corner(full_path, PathPoint(float(p.row), float(p.col)))

        _append_connection_corner(full_path, all_segments[0][0])
        _extend_path_corners(full_path, all_segments[0][1:])
        last_added_idx = 0
        # full_path 会在线压缩共线点，不能再用“原段点数”反推段首索引；
        # 第一段严格从现场起点开始，因此段首就是索引0。
        last_segment_start = 0
        turn_by_line_id = 0
        same_line_bridges = 0
        fallback_bridges = 0

        for i in range(1, len(all_segments)):
            seg = all_segments[i]
            a = full_path[-1]
            b = seg[0]
            prev_line = segment_line_ids[last_added_idx]
            curr_line = segment_line_ids[i]
            gap = abs(a.row - b.row) + abs(a.col - b.col)

            if gap <= 1.5 * max(step, 1.0):
                last_segment_start = len(full_path)
                _append_connection_corner(full_path, seg[0])
                _extend_path_corners(full_path, seg[1:])
                last_added_idx = i
                continue

            connected = False
            if (curr_line == prev_line + 1 and
                    abs(segment_base_s[i] - segment_base_s[last_added_idx])
                    <= 1.5 * path_spacing):
                # 只允许真实相邻 line_id 做跨线掉头；角点使用原始 base_s，
                # 完全不使用避障后发生偏移的端点法向坐标。
                prev_s = segment_base_s[last_added_idx]
                curr_s = segment_base_s[i]
                def _pt_t(p):
                    return p.col * dx + p.row * dy

                def _pt_s(p):
                    return p.col * nx + p.row * ny

                # 以当前真实端点判断位于哪一侧，不能只依赖奇偶号；扫描线
                # 被裁剪/跳过后奇偶可能失配，进而产生先越界再回头的小尖钩。
                a_t = _pt_t(a)
                b_t = _pt_t(b)
                side_max = abs(a_t - d_max) <= abs(a_t - d_min)
                # 掉头上限取两条已留边扫描线的共同内侧端点，不能再用原始
                # d_min/d_max，否则会把2.2m留边重新拉回边界。
                turn_cap = min(a_t, b_t) if side_max else max(a_t, b_t)

                max_inset = max(2, min(12, int(round(0.4 * path_spacing))))
                for inset_i in range(2, max_inset + 1):
                    inset = float(inset_i)
                    turn_t = turn_cap - inset if side_max else turn_cap + inset
                    # 扫描端点在旋转坐标系中保持90°直角掉头。
                    turn_a = PathPoint(turn_t * dy + prev_s * ny,
                                       turn_t * dx + prev_s * nx)
                    turn_b = PathPoint(turn_t * dy + curr_s * ny,
                                       turn_t * dx + curr_s * nx)

                    tail = len(full_path) - 1
                    if side_max:
                        while tail > last_segment_start and _pt_t(full_path[tail]) > turn_t:
                            tail -= 1
                    else:
                        while tail > last_segment_start and _pt_t(full_path[tail]) < turn_t:
                            tail -= 1
                    # 90°端点只能从原扫描基线进入；先裁掉尚未回正的避障斜坡，
                    # 避免“斜坡尾巴 + 直角点”形成一格尖刺。
                    while (tail > last_segment_start and
                           abs(_pt_s(full_path[tail]) - prev_s) > 0.15):
                        tail -= 1

                    head = 0
                    if side_max:
                        while head + 1 < len(seg) and _pt_t(seg[head]) > turn_t:
                            head += 1
                    else:
                        while head + 1 < len(seg) and _pt_t(seg[head]) < turn_t:
                            head += 1
                    while (head + 1 < len(seg) and
                           abs(_pt_s(seg[head]) - curr_s) > 0.15):
                        head += 1

                    if (abs(_pt_s(full_path[tail]) - prev_s) > 0.15 or
                            abs(_pt_s(seg[head]) - curr_s) > 0.15):
                        continue

                    if (_bridge_line_free(full_path[tail], turn_a)
                            and _bridge_line_free(turn_a, turn_b)
                            and _bridge_line_free(turn_b, seg[head])):
                        del full_path[tail + 1:]
                        # 边界原端点可能比安全直角点多伸出数格；直接截到直角点，
                        # 禁止先触边再反向折回形成0.1°~5°可见的小尖钩。
                        if tail > last_segment_start:
                            full_path[-1] = turn_a
                        else:
                            # 稀疏角点模式下本扫描段可能只剩“起点+终点”；
                            # 绝不能为截边尖钩而覆盖现场起点。
                            _append_path_corner(full_path, turn_a)
                        _append_path_corner(full_path, turn_b)
                        last_segment_start = len(full_path) - 1
                        _extend_path_corners(full_path, seg[head:])
                        last_added_idx = i
                        turn_by_line_id += 1
                        connected = True
                        break

            if connected:
                continue

            # 同一 line_id 只能是线内障碍断段；非相邻 line_id 也不得猜测掉头。
            mid = self._route_astar(grid_map, a, b)
            head_already_added = False
            if mid is None:
                if not _append_connection_with_backtrack(
                        full_path, seg[0], last_segment_start):
                    continue
                head_already_added = True
            else:
                for p_mid in mid:
                    _append_connection_corner(
                        full_path,
                        PathPoint(float(p_mid.row), float(p_mid.col)))
            last_segment_start = (len(full_path) - 1
                                  if head_already_added else len(full_path))
            if (math.hypot(full_path[-1].row - seg[0].row,
                           full_path[-1].col - seg[0].col) > 1e-7 and
                    not _append_connection_corner(full_path, seg[0])):
                # 暂存段首，最终输出级几何校验会重新分解；不能因此丢掉整条作业线。
                _append_corner(full_path, seg[0])
            _extend_path_corners(full_path, seg[1:])
            if curr_line == prev_line:
                same_line_bridges += 1
            else:
                fallback_bridges += 1
            last_added_idx = i

        print(f"   [line_id连接] 相邻线掉头={turn_by_line_id}, "
              f"线内桥接={same_line_bridges}, 其他回退={fallback_bridges}")

        # 终点保证：强制 A* 走到终点，不连通则不追加（绝不画直线）
        if end_point is not None and full_path:
            last = full_path[-1]
            ep0 = Point(int(round(end_point.row)), int(round(end_point.col)))
            if abs(last.row - ep0.row) + abs(last.col - ep0.col) > 1:
                if (0 <= ep0.row < H and 0 <= ep0.col < W
                        and grid_map[ep0.row, ep0.col] == 0):
                    mid = self._route_astar(grid_map, last, ep0)
                    if mid is not None:
                        for p in mid:
                            _append_connection_corner(full_path, PathPoint(float(p.row), float(p.col)))
                        _append_connection_corner(full_path, PathPoint(float(ep0.row), float(ep0.col)))

        # 安全局部平滑：直线段均值不变，只圆滑边界掉头、A* 栅格锯齿和
        # 梯形避障折点。每个候选���及相邻短线段都重新做边界/碰撞检查；
        # 不安全则保留原点，绝不以“平滑”为代价切入柱子或越界。
        def _line_free(a, b, sample_step=0.5):
            dist = math.hypot(b.col - a.col, b.row - a.row)
            count = max(1, int(math.ceil(dist / sample_step)))
            for q in range(1, count + 1):
                u = q / count
                if not is_free(a.col + (b.col - a.col) * u,
                               a.row + (b.row - a.row) * u):
                    return False
            return True

        def _smooth_safe(src, passes=2):
            if len(src) < 5:
                return src
            radius = max(4, min(24, int(round(
                0.25 * path_spacing / max(step, 0.5)))))
            cur_path = list(src)
            for _ in range(passes):
                old = cur_path
                new = [old[0]]                 # 起点精确保持
                for idx in range(1, len(old) - 1):
                    # 起终点邻域保持原短接入/退出段，避免固定端点与平滑点错位。
                    if idx <= radius or idx >= len(old) - 1 - radius:
                        new.append(old[idx])
                        continue
                    lo = max(0, idx - radius)
                    hi = min(len(old) - 1, idx + radius)
                    sw = sr = sc = 0.0
                    for q in range(lo, hi + 1):
                        w = float(radius + 1 - abs(q - idx))
                        sw += w
                        sr += w * old[q].row
                        sc += w * old[q].col
                    cand = PathPoint(sr / sw, sc / sw)
                    if (is_free(cand.col, cand.row)
                            and _line_free(new[-1], cand)
                            and _line_free(cand, old[idx + 1])):
                        new.append(cand)
                    else:
                        new.append(old[idx])
                new.append(old[-1])             # 终点精确保持
                cur_path = new
            return cur_path

        # RK3588 稀疏角点模式：跳过 O(N*窗口*轮次) 的全路径平滑。
        # 原始路径已做膨胀、侧让和 A* 安全连接，直接进入安全角点提取。
        # 二次安全去毛刺：处理接近 0°/90° 时，A* 整数格回退造成的
        # 1~3 格短凸点。只移动高频局部异常点，不拉直正常 U 形掉头。
        def _despike_safe(src, passes=6):
            if len(src) < 5:
                return src
            cur_path = list(src)
            endpoint_guard = max(6, int(round(
                0.25 * path_spacing / max(step, 0.5))))
            for _ in range(passes):
                old = cur_path
                new = list(old)
                for idx in range(endpoint_guard, len(old) - endpoint_guard):
                    cand = PathPoint(
                        0.5 * (old[idx - 1].row + old[idx + 1].row),
                        0.5 * (old[idx - 1].col + old[idx + 1].col))
                    deviation = math.hypot(cand.row - old[idx].row,
                                           cand.col - old[idx].col)
                    if not (0.15 < deviation <= 3.0):
                        continue
                    if (is_free(cand.col, cand.row)
                            and _line_free(old[idx - 1], cand)
                            and _line_free(cand, old[idx + 1])):
                        new[idx] = cand
                cur_path = new
            return cur_path

        # 角点已在采样、A* 接续和段拼接时在线生成，不再做全路径 RDP。
        normalized = []
        for point in full_path:
            if not normalized:
                normalized.append(point)
                continue
            a = normalized[-1]
            vx, vy = point.col - a.col, point.row - a.row
            dt, ds = vx * dx + vy * dy, vx * nx + vy * ny
            is_obstacle_angle = (
                abs(abs(ds) - corner_tangent * abs(dt)) <= 1e-5)
            at = a.col * dx + a.row * dy
            bt = point.col * dx + point.row * dy
            boundary_band = max(3.0, 0.75 * path_spacing)
            boundary_obstacle_angle = is_obstacle_angle and (
                (abs(at - d_min) <= boundary_band and abs(bt - d_min) <= boundary_band) or
                (abs(at - d_max) <= boundary_band and abs(bt - d_max) <= boundary_band))
            if boundary_obstacle_angle:
                # 边界掉头禁用柱子折角：优先还原成d/n正交狗腿。
                ass = a.col * nx + a.row * ny
                bs = point.col * nx + point.row * ny
                corners = [
                    PathPoint(bt * dy + ass * ny, bt * dx + ass * nx),
                    PathPoint(at * dy + bs * ny, at * dx + bs * nx),
                ]
                folded = False
                for corner in corners:
                    if (_bridge_line_free(a, corner) and
                            _bridge_line_free(corner, point)):
                        _append_corner(normalized, corner)
                        _append_corner(normalized, point)
                        folded = True
                        break
                if not folded:
                    _append_corner(normalized, point)
            elif (abs(ds) <= 1e-5 or abs(dt) <= 1e-5 or
                  is_obstacle_angle):
                _append_corner(normalized, point)
            else:
                if not _append_connection_corner(normalized, point):
                    _append_corner(normalized, point)
        full_path = normalized

        # 首末作业线只能从现场起点向内、并从区域内到现场终点；删除选点
        # 外侧因边界量化/掉头重构遗留的短残段。
        endpoint_tol = 0.20
        if (start_point is not None and full_path and
                not reposition_to_forced_start):
            kept = [full_path[0]]
            idx = 1
            while idx < len(full_path):
                ps = full_path[idx].col * nx + full_path[idx].row * ny
                if abs(ps - s_start) > endpoint_tol:
                    break
                pt = full_path[idx].col * dx + full_path[idx].row * dy
                if ((first_ascending and pt >= start_t - 1e-6) or
                        (not first_ascending and pt <= start_t + 1e-6)):
                    _append_corner(kept, full_path[idx])
                idx += 1
            for point in full_path[idx:]:
                _append_corner(kept, point)
            full_path = kept
        if end_point is not None and full_path:
            cut = len(full_path) - 2
            while cut >= 0:
                ps = full_path[cut].col * nx + full_path[cut].row * ny
                if abs(ps - s_end) > endpoint_tol:
                    break
                pt = full_path[cut].col * dx + full_path[cut].row * dy
                valid = ((wanted_last_ascending and pt <= end_t + 1e-6) or
                         (not wanted_last_ascending and pt >= end_t - 1e-6))
                if not valid:
                    del full_path[cut]
                cut -= 1
            exact_end = PathPoint(float(end_point.row), float(end_point.col))
            last = full_path[-1]
            if math.hypot(last.row - exact_end.row,
                          last.col - exact_end.col) <= 1e-7:
                full_path[-1] = exact_end
            elif not _append_connection_with_backtrack(
                    full_path, exact_end, max(0, len(full_path) - 12)):
                # A* 可能因栅格对角接触而拒绝回接；此处仍只允许旋转坐标系
                # 的直角或“配置折角+轴向”安全连接。禁止用终点覆盖最后一个角点，
                # 否则会重新制造任意斜率长线和终点重复覆盖。
                print(f"   [严格终点失败] 无法用直角/{corner_angle_deg:g}°"
                      f"安全连接到现场终点: "
                      f"({last.row:.3f},{last.col:.3f}) -> "
                      f"({exact_end.row:.3f},{exact_end.col:.3f})")
                return []
        print(f"   [RK3588在线角点] 输出 {len(full_path)} 点（无规划后抽稀）")
        return full_path

    def plan_subregion_boustrophedon(self, grid_map, boundary_polygon, direction,
                                     path_spacing, direction_angle=None,
                                     start_point=None, end_point=None):

        rows = [p.row for p in boundary_polygon]
        cols = [p.col for p in boundary_polygon]
        if not rows: return []
        min_row, max_row = max(0, min(rows)), min(grid_map.shape[0] - 1, max(rows))
        min_col, max_col = max(0, min(cols)), min(grid_map.shape[1] - 1, max(cols))
        region_mask = ObstacleProcessor.polygon_to_mask(
            boundary_polygon, grid_map.shape, radius=-0.5
        )

        all_segments = []
        is_x = _direction_axis(direction) == 'x'

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
                    scan_start_col, scan_end_col, True,
                    region_mask=region_mask
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

            print(f"   [扫描���] 纵向扫描: 共{len(scan_lines)}条")

            # 控制每条扫���线内部的起止点
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
                    scan_start_row, scan_end_row, False,
                    region_mask=region_mask
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
            if region_mask[point.row, point.col]:
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
                        region_mask[end_point.row, end_point.col]):
                    filtered_path.append(end_point)
                    print(f"   [终点添加] 终点已添加到路径末尾")

        return filtered_path

    def _get_scan_segments(self, grid_map, poly, fixed_idx, start_idx, end_idx, is_row_scan,
                           region_mask=None):
        """
        【修复版】获取扫描线段，增加严格的边界检查
        """
        segments = []
        current_segment = []
        rows, cols = grid_map.shape

        for moving_idx in range(start_idx, end_idx + 1, 6):
            r, c = (fixed_idx, moving_idx) if is_row_scan else (moving_idx, fixed_idx)

            # 检查是否在网格范围内
            if not (0 <= r < rows and 0 <= c < cols):
                if current_segment:
                    segments.append(current_segment)
                    current_segment = []
                continue

            # 检查是否有障碍物 + 严格的多边形内部检查
            if region_mask is not None:
                is_valid = (grid_map[r, c] == 0) and region_mask[r, c]
            else:
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
        mid = self._route_astar(grid_map, safe_start, safe_goal)
        if mid is None:
            return []
        return [safe_start, *mid, safe_goal]

    def _astar_search(self, grid_map, start, goal):
        start_node = (start.row, start.col)
        goal_node = (goal.row, goal.col)
        if start_node == goal_node: return [start]

        rows, cols = grid_map.shape
        open_set = []
        heappush = heapq.heappush
        heappop = heapq.heappop
        heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        goal_row, goal_col = goal_node

        movements = [
            (0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        max_steps = 500000
        steps = 0

        while open_set and steps < max_steps:
            steps += 1
            current = heappop(open_set)[1]

            if current == goal_node:
                # 直接沿父链提取方向变化点，不构造逐格 A* 路径。
                path = [Point(current[0], current[1])]
                previous_dir = None
                while current in came_from:
                    parent = came_from[current]
                    direction = (parent[0] - current[0], parent[1] - current[1])
                    if previous_dir is not None and direction != previous_dir:
                        path.append(Point(current[0], current[1]))
                    previous_dir = direction
                    current = parent
                path.append(Point(current[0], current[1]))
                return path[::-1]

            current_row, current_col = current
            for dr, dc, cost in movements:
                nr, nc = current_row + dr, current_col + dc

                if 0 <= nr < rows and 0 <= nc < cols:
                    if grid_map[nr, nc] == 1: continue

                    if abs(dr) == 1 and abs(dc) == 1:
                        if grid_map[current_row + dr, current_col] == 1 or grid_map[current_row, current_col + dc] == 1:
                            continue

                    new_g = g_score[current] + cost
                    neighbor = (nr, nc)
                    if neighbor not in g_score or new_g < g_score[neighbor]:
                        g_score[neighbor] = new_g
                        priority = new_g + abs(nr - goal_row) + abs(nc - goal_col)
                        heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        print(f"   [A*警告] 未找到路径: {start} -> {goal}")
        return []


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
        if self.robot_config.endpoint_margin < 0:
            raise ValueError("endpoint_margin must be >= 0")
        if self.robot_config.output_point_spacing < 0:
            raise ValueError("output_point_spacing must be >= 0")
        if self.robot_config.aligned_obstacle_inflation < 0:
            raise ValueError("aligned_obstacle_inflation must be >= 0")
        if self.robot_config.aligned_obstacle_max_extent <= 0:
            raise ValueError("aligned_obstacle_max_extent must be > 0")
        if not (0.0 < self.robot_config.obstacle_corner_angle_deg < 90.0):
            raise ValueError("obstacle_corner_angle_deg must be between 0 and 90")
        if self.robot_config.obstacle_avoidance_distance < 0:
            raise ValueError("obstacle_avoidance_distance must be >= 0")

    @staticmethod
    def _densify_path(path, spacing_pixels):
        """补点前先清除阶段拼接产生的全局A→B→A针状支路。"""
        if len(path) < 2 or spacing_pixels <= 1e-9:
            return list(path)
        corners = list(path)

        dense = [corners[0]]
        for a, b in zip(corners, corners[1:]):
            dr, dc = b.row - a.row, b.col - a.col
            dist = math.hypot(dr, dc)
            if dist <= 1e-9:
                continue
            k = 1
            while k * spacing_pixels < dist - 1e-9:
                u = (k * spacing_pixels) / dist
                dense.append(PathPoint(
                    float(a.row + u * dr), float(a.col + u * dc),
                    0.0, 'intermediate', a.path_type or b.path_type))
                k += 1
            dense.append(b)
        return dense

    @staticmethod
    def _path_collision_free(path, collision_grid, sample_step=0.5):
        """用原始轮廓膨胀图复检整条路径；首末现场点本身不作静默移动。"""
        if not path:
            return False
        h, w = collision_grid.shape
        for seg_idx, (a, b) in enumerate(zip(path, path[1:])):
            dist = math.hypot(b.row - a.row, b.col - a.col)
            count = max(1, int(math.ceil(dist / sample_step)))
            for q in range(1, count + 1):
                if seg_idx == len(path) - 2 and q == count:
                    continue
                u = q / count
                row = int(round(a.row + (b.row - a.row) * u))
                col = int(round(a.col + (b.col - a.col) * u))
                if not (0 <= row < h and 0 <= col < w):
                    return False
                if collision_grid[row, col] != 0:
                    return False
        return True

    def _plan_with_cpp_backend(self, inflated_grid, stages, real_to_grid_ratio,
                               effective_spacing, stage_planning_grids=None):
        if _mst27_cpp is None or os.environ.get("MST27_DISABLE_CPP") == "1":
            return None

        spacing_pixels = max(1, int(round(effective_spacing * real_to_grid_ratio)))
        grid_for_cpp = np.ascontiguousarray(inflated_grid, dtype=np.int32)
        stage_specs = []

        for stage_index, stage in enumerate(stages):
            endpoint_margin_m = (self.robot_config.endpoint_margin
                                 if stage.endpoint_margin is None
                                 else stage.endpoint_margin)
            endpoint_margin_pixels = max(
                0.0, float(endpoint_margin_m) * float(real_to_grid_ratio))
            region_mask = ObstacleProcessor.polygon_to_mask(
                stage.boundary_polygon,
                inflated_grid.shape,
                radius=-0.5
            )
            spec = {
                "stage_id": int(stage.stage_id),
                "boundary": [(int(p.row), int(p.col)) for p in stage.boundary_polygon],
                "start_point": None if stage.start_point is None else (
                    int(stage.start_point.row),
                    int(stage.start_point.col)
                ),
                "end_point": (int(stage.end_point.row), int(stage.end_point.col)),
                "direction": stage.direction,
                "direction_angle": None if stage.direction_angle is None else float(stage.direction_angle),
                "endpoint_margin_pixels": endpoint_margin_pixels,
                "obstacle_corner_angle_deg": float(
                    self.robot_config.obstacle_corner_angle_deg),
                "obstacle_avoidance_pixels": float(
                    self.robot_config.obstacle_avoidance_distance) *
                    float(real_to_grid_ratio),
                "region_mask": np.ascontiguousarray(region_mask, dtype=np.bool_)
            }
            if stage_planning_grids is not None:
                spec["planning_grid"] = np.ascontiguousarray(
                    stage_planning_grids[stage_index], dtype=np.int32)
            stage_specs.append(spec)

        try:
            output_spacing_pixels = max(
                0.0, self.robot_config.output_point_spacing * real_to_grid_ratio)
            cpp_path = _mst27_cpp.plan_core(
                grid_for_cpp, stage_specs, spacing_pixels, output_spacing_pixels)
        except Exception as exc:
            print(f"[C++后端] 调用失败，回退到Python实现: {exc}")
            return None

        print(f"[C++后端] 使用C++规划内核，间距: {spacing_pixels}像素")
        return [
            PathPoint(float(row), float(col), 0, 'intermediate', path_type)
            for row, col, path_type in cpp_path
        ]

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
        print("输入参数详���")
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
        direction_desc = "横向扫描(固定row,沿col移动)" if _direction_axis(global_direction) == 'x' else "纵向扫描(固定col,沿row移动)"
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
        print(f"  ├─ inflation_radius: {self.robot_config.inflation_radius}m")
        print(f"  ├─ aligned_obstacle_inflation: "
              f"{self.robot_config.aligned_obstacle_inflation}m")
        print(f"  ├─ aligned_obstacle_max_extent: "
              f"{self.robot_config.aligned_obstacle_max_extent}m")
        print(f"  ├─ obstacle_corner_angle_deg: "
              f"{self.robot_config.obstacle_corner_angle_deg}°")
        print(f"  ��─ obstacle_avoidance_distance: "
              f"{self.robot_config.obstacle_avoidance_distance}m")

        print("=" * 80)
        print("║ 参数打印完毕，开始执行规划...")
        print("=" * 80 + "\n")

        # ============================================================
        # 原有的函数逻辑从这里开始
        # ============================================================

        print("\n" + "=" * 60)
        print("开始路径规划（修复版 v4.7.1）")
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
                print(f"  → ���碍物: {len(global_obstacle_polygons)} 个")
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
            print(f"  ✓ 自动创建阶段1，覆��整个全局区域")
        else:
            print(f"\n[模式] �����阶段规划，共{len(stages)}个子区域")

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
            print(f"  ✓ 共��理 {total_stage_obstacles} 个障碍物多边形")
        else:
            print(f"  无阶段特定障碍物")

        # 保留原始轮廓图：用于机器人本体安全半径膨胀及最终碰撞复检。
        raw_obstacle_grid = np.asarray(grid_map, dtype=int).copy()
        inflation_pixels = int(self.robot_config.inflation_radius * real_to_grid_ratio)
        print(f"\n[参数] 机器人宽度: {self.robot_config.width}m, 长度: {self.robot_config.length}m")
        print(f"[参数] 膨胀半径: {self.robot_config.inflation_radius}m ({inflation_pixels}像素)")

        inflated_grid = ObstacleProcessor.inflate_obstacles(grid_map, inflation_pixels)

        aligned_inflation_pixels = max(0, int(round(
            self.robot_config.aligned_obstacle_inflation * real_to_grid_ratio)))
        aligned_max_extent_pixels = max(1, int(round(
            self.robot_config.aligned_obstacle_max_extent * real_to_grid_ratio)))
        print(f"[参数] 角度对齐矩形膨胀: "
              f"{self.robot_config.aligned_obstacle_inflation}m "
              f"({aligned_inflation_pixels}像素)")
        print(f"[参数] 柱子折角: "
              f"{self.robot_config.obstacle_corner_angle_deg}°")
        print(f"[参数] 路径规划躲避距离: "
              f"{self.robot_config.obstacle_avoidance_distance}m")
        stage_planning_grids = []
        for stage in stages:
            stage_angle = (float(stage.direction_angle)
                           if stage.direction_angle is not None
                           else (0.0 if _direction_axis(stage.direction) == 'x' else 90.0))
            stage_grid = ObstacleProcessor.build_angle_aligned_inflated(
                raw_obstacle_grid,
                stage_angle,
                aligned_inflation_pixels,
                base_inflation_pixels=inflation_pixels,
                max_component_extent_pixels=aligned_max_extent_pixels)
            stage_planning_grids.append(stage_grid)

        effective_spacing = self.robot_config.path_spacing * (1 - self.robot_config.overlap_ratio)
        print(f"[参数] 路径间��: {self.robot_config.path_spacing}m, 重叠: {self.robot_config.overlap_ratio}")
        print(f"[参数] 有效间距: {effective_spacing:.2f}m")

        print(f"[说明] 仅规划已定义的子区域，无需完全覆盖全局区域")

        cpp_path = self._plan_with_cpp_backend(
            inflated_grid,
            stages,
            real_to_grid_ratio,
            effective_spacing,
            stage_planning_grids
        )
        if cpp_path is not None:
            if not self._path_collision_free(cpp_path, inflated_grid):
                print("[最终碰撞复检] C++路径触碰原始膨胀障碍，拒绝输出")
                return []
            print("[最终碰撞复检] 通过（原始轮廓膨胀图）")
            print("\n" + "=" * 60)
            print(f"规划完成！总路径点数: {len(cpp_path)}")
            print("=" * 60 + "\n")
            return cpp_path

        complete_path = []
        last_stage_end = None

        for i, stage in enumerate(stages):
            print(f"\n--- 阶段 {stage.stage_id} ---")
            planning_grid = stage_planning_grids[i]

            # 起点选择
            if stage.start_point:
                curr_start = stage.start_point
                print(f"[起点] 使用指定起点: (x={curr_start.col}, y={curr_start.row})")
            else:
                curr_start = self.planner_core._find_farthest_corner(stage.boundary_polygon, stage.end_point)
                print(f"[起点] 自动选择最远角点: (x={curr_start.col}, y={curr_start.row})")
                curr_start = self.planner_core._find_nearest_free_point(planning_grid, curr_start)

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

            if stage.direction_angle is not None:
                print(f"[规划] 任意角度模式: {float(stage.direction_angle):.4f}°")
                region_pts = self.planner_core.plan_subregion_oriented(
                    planning_grid,
                    stage.boundary_polygon,
                    float(stage.direction_angle),
                    spacing_pixels,
                    step=0.5,
                    start_point=curr_start,
                    end_point=stage.end_point,
                    endpoint_margin=(self.robot_config.endpoint_margin
                                     if stage.endpoint_margin is None
                                     else stage.endpoint_margin) * real_to_grid_ratio,
                    first_travel_sign=_direction_travel_sign(stage.direction),
                )
            else:
                # 轴向模式也统一走角点直出的任意角度内核，避免旧规划器
                # 逐格生成点以及A*栅格锯齿。x=0°，y=90°。
                axis_angle = 0.0 if _direction_axis(stage.direction) == 'x' else 90.0
                region_pts = self.planner_core.plan_subregion_oriented(
                    planning_grid,
                    stage.boundary_polygon,
                    axis_angle,
                    spacing_pixels,
                    step=1.0,
                    start_point=curr_start,
                    end_point=stage.end_point,
                    endpoint_margin=(self.robot_config.endpoint_margin
                                     if stage.endpoint_margin is None
                                     else stage.endpoint_margin) * real_to_grid_ratio,
                    first_travel_sign=_direction_travel_sign(stage.direction),
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

        output_spacing_pixels = max(
            0.0, self.robot_config.output_point_spacing * real_to_grid_ratio)
        complete_path = self._densify_path(complete_path, output_spacing_pixels)
        if not self._path_collision_free(complete_path, inflated_grid):
            print("[最终碰撞复检] Python路径触碰原始膨胀障碍，拒绝输出")
            return []
        print("[最终碰撞复检] 通过（原始轮廓膨胀图）")
        print(f"[最终补点] 间距={self.robot_config.output_point_spacing:.3f}m, "
              f"输出点数={len(complete_path)}")
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

def _visualize_tilted(inflated, obstacles, region, path, sp, ep, angle, res, out_path):
    """渲染倾斜矩形框+柱子场景：浅蓝=区域, 橙=机器人半径安全环, 黑=真实柱, 蓝线=路径。"""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    H, W = inflated.shape
    img = np.full((H, W, 3), 255, dtype=np.uint8)
    img[region == 1] = (226, 236, 247)
    img[(inflated == 1) & (obstacles == 0)] = (255, 214, 170)
    img[obstacles == 1] = (30, 30, 36)
    fig, ax = plt.subplots(figsize=(12, 12 * H / max(W, 1)))
    ax.imshow(img, origin="upper")
    if path:
        ax.plot([p.col for p in path], [p.row for p in path], "-",
                color="#0060d0", linewidth=0.6)
    ax.plot([sp.col], [sp.row], "o", color="#00aa00", markersize=8)
    ax.plot([ep.col], [ep.row], "o", color="#dc0000", markersize=8)
    ax.set_title("[v4.7.1 corner60 rotated-frame] tilt %.2f deg  %dcm/cell  pts=%d  (orange=aligned clearance, black=raw obstacle)"
                 % (angle, int(round(100.0 / res)), len(path)))
    ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis("off")
    fig.tight_layout(); fig.savefig(out_path, dpi=140); plt.close(fig)
    print(f"[可视化] 已保存 {out_path}")


def demo_fixed(direction_angle=37.58):
    """现行测试场景：约 5000㎡ 的矩形框整体倾�� direction_angle 度（含外框），
    框内随机布置 0.8×0.8 / 0.6×0.6 m 柱子，行列间距随机 6m 或 8m；
    分辨率 5cm/格；按机器人半径膨胀后做任意角度弓字形规划。
    direction_angle: 规划/倾斜角（度），支持任意 4 位小数；None → 用 0°(轴向)。
    """
    import math, time, os, random
    print("=" * 64)
    print("  mst27 规划内核版本: v4.7.1  (柱子/墙体分类 + 60度柱边折角 + 旋转场地框)")
    print("  当前测试场景: 稀疏角点，越界 0，碰撞 0，中间间距固定 1.5m")
    print("=" * 64)
    import numpy as np
    from scipy import ndimage

    angle = 0.0 if direction_angle is None else float(direction_angle)
    demo_start = time.perf_counter()
    print("=" * 70)
    print(f"[测试场景] 5000㎡矩��框整体倾斜 {angle:.4f}°，随机柱子(0.8/0.6m, 间距6/8m)，5cm/格")
    print("=" * 70)

    RES = 20.0            # 格/米 → 5cm/格
    WM, HM = 80.0, 62.5   # 80×62.5 = 5000 ㎡
    PATH_SPACING_M = 1.5
    STEP = 0.5
    MARGIN = 20
    robot_config = RobotConfig(width=1.3, length=2.0, path_spacing=PATH_SPACING_M,
                               turning_radius=1.0, overlap_ratio=0.15,
                               inflation_radius=0.65,
                               aligned_obstacle_inflation=1.0,
                               obstacle_corner_angle_deg=45.0,
                               endpoint_margin=2,  # 米：X左右/Y上下/任意角度沿扫描方向留边
                               output_point_spacing=0.3)  # 米：规划完成后每30cm补点

    random.seed(20260715)
    th = math.radians(angle); ct, sn = math.cos(th), math.sin(th)
    cx, cy = WM / 2, HM / 2
    def rot(x, y):
        return ct * (x - cx) - sn * (y - cy), sn * (x - cx) + ct * (y - cy)
    corners_local = [(0, 0), (WM, 0), (WM, HM), (0, HM)]
    rcs = [rot(x, y) for x, y in corners_local]
    minx = min(p[0] for p in rcs); maxx = max(p[0] for p in rcs)
    miny = min(p[1] for p in rcs); maxy = max(p[1] for p in rcs)
    def to_grid(x, y):
        xr, yr = rot(x, y)
        return (xr - minx) * RES + MARGIN, (yr - miny) * RES + MARGIN
    W = int(math.ceil((maxx - minx) * RES)) + 2 * MARGIN
    H = int(math.ceil((maxy - miny) * RES)) + 2 * MARGIN

    # 随机柱子（局部坐标，米）
    columns = []
    y = 4.0
    while y <= HM - 4.0:
        x = 4.0
        while x <= WM - 4.0:
            columns.append((x, y, random.choice([0.8, 0.6])))
            x += random.choice([6.0, 8.0])
        y += random.choice([6.0, 8.0])

    # 栅格化（向量化逆旋转）
    rr, cc = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
    xr = (cc - MARGIN) / RES + minx; yr = (rr - MARGIN) / RES + miny
    xl = ct * xr + sn * yr + cx; yl = -sn * xr + ct * yr + cy
    region = (xl >= 0) & (xl <= WM) & (yl >= 0) & (yl <= HM)
    obstacles = np.zeros((H, W), dtype=np.uint8)
    for (kx, ky, ks) in columns:
        h = ks / 2.0
        obstacles[(np.abs(xl - kx) <= h) & (np.abs(yl - ky) <= h)] = 1

    # 原始轮廓膨胀：仅用于最终碰撞复检。
    rad = int(round(robot_config.inflation_radius * RES))
    yy, xx = np.ogrid[-rad:rad + 1, -rad:rad + 1]
    disk = (xx * xx + yy * yy) <= rad * rad
    original_inflated = ndimage.binary_dilation(
        obstacles, structure=disk).astype(np.uint8)
    original_inflated[~region] = 0
    aligned_rad = int(round(robot_config.aligned_obstacle_inflation * RES))
    inflated = ObstacleProcessor.build_angle_aligned_inflated(
        obstacles, angle, aligned_rad)
    inflated[~region] = 0                 # 角度矩形膨胀安全区（橙色）
    grid_for_plan = inflated.copy()
    grid_for_plan[~region] = 1            # 边界外一律视为障���，A* 连接/桥接不会越界

    boundary = [Point(int(round(to_grid(x, y)[1])), int(round(to_grid(x, y)[0])))
                for (x, y) in corners_local]
    gx0, gy0 = to_grid(2.0, 2.0);          sp = Point(int(round(gy0)), int(round(gx0)))
    gx1, gy1 = to_grid(WM - 2.0, HM - 2.0); ep = Point(int(round(gy1)), int(round(gx1)))
    PS = max(1, int(round(PATH_SPACING_M * RES)))

    print(f"栅格 {H}×{W} ({H*W/1e6:.2f}M) | 区域格 {int(region.sum())} | 柱子 {len(columns)} 根 "
          f"| 障碍格 {int(obstacles.sum())}→{int(inflated.sum())}"
          f"(角度矩形膨胀{aligned_rad}格) | 线距 {PS}格 步长 {STEP}")

    t0 = time.perf_counter()
    if _mst27_cpp is not None:
        cpp_ver = getattr(_mst27_cpp, "__version__", "unknown/旧模块")
        print(f"[规划引擎] C++ _mst27_cpp.plan_core  version={cpp_ver}")
        stage_specs = [{
            "stage_id": 1,
            "boundary": [(p.row, p.col) for p in boundary],
            "start_point": (sp.row, sp.col),
            "end_point": (ep.row, ep.col),
            "direction": "x",
            "direction_angle": float(angle),
            "obstacle_corner_angle_deg": float(
                robot_config.obstacle_corner_angle_deg),
            "obstacle_avoidance_pixels": float(
                robot_config.obstacle_avoidance_distance) * RES,
            "region_mask": np.ascontiguousarray(region, dtype=np.bool_),
        }]
        output_spacing_pixels = max(
            0.0, robot_config.output_point_spacing * RES)
        raw_path = _mst27_cpp.plan_core(
            np.ascontiguousarray(grid_for_plan, dtype=np.int32), stage_specs, PS,
            output_spacing_pixels)
        path = [PathPoint(float(r), float(c), path_type=str(pt))
                for r, c, pt in raw_path]
    else:
        print("[规划引擎] Python fallback（未加载 _mst27_cpp，速度会明显变慢）")
        print(f"[C++加载失败] {_mst27_cpp_import_error}")
        planner = ObstacleAwareBoustrophedonPlanner(robot_config)
        path = planner.plan_subregion_oriented(grid_for_plan, boundary, angle, PS, step=STEP,
                                               start_point=sp, end_point=ep)
        output_spacing_pixels = max(
            0.0, robot_config.output_point_spacing * RES)
        path = MultiStagePathPlanner._densify_path(path, output_spacing_pixels)
    elapsed = time.perf_counter() - t0
    print(f"[规划完成] 用时 {elapsed*1000:.1f} ms，路径点 {len(path)}")
    if not MultiStagePathPlanner._path_collision_free(path, original_inflated):
        print("[最终碰撞复检] 演示路径触碰原始膨胀障碍，拒绝输出")
        path = []
    else:
        print("[最终碰撞复检] 通过（原始轮廓膨胀图）")

    output_dir = os.getcwd()
    _visualize_tilted(inflated, obstacles, region, path, sp, ep, angle, RES,
                      os.path.join(output_dir, "路径规划_倾斜柱子.png"))
    export_path_to_json(path, "路径点输出.json", output_dir=output_dir)
    print(f"[运行时间] 总耗时 {time.perf_counter() - demo_start:.3f} 秒")
    return path


# ============================================================
# 主程序入口
# ============================================================

if __name__ == "__main__":
    print("""
╔════════════════════════════════════════════��═════════════════════════╗
║ 四局你好！                     
╚══════════════════════════════════════════════════════════════════════╝
    """)

    import sys
    # ====================================================================
    #  【输入规划角度】在这里修改扫描角度（度），支持任意 4 位小数
    #    - 设为 None 则用轴向扫描(x/y)；设为数值则用任意角度规划
    #    - 也可命令行传入：  python3 mst27.py 37.1234
    # ====================================================================
    INPUT_ANGLE = 0        # ← 在此修改想要的规划角度，或改成 None
    if len(sys.argv) > 1:
        INPUT_ANGLE = float(sys.argv[1])

    path = demo_fixed(direction_angle=INPUT_ANGLE)

    print("\n" + "=" * 70)
    print("✅ 演示完成!")
    print("=" * 70)
    print("\n输出文件:")
    print("  1. 路径点输出.json")
    print("  2. 路径规划.png")
    print("\n" + "=" * 70)
