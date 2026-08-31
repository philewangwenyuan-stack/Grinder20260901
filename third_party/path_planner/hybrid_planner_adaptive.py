import math
import heapq
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any, Union

# =========================
# 兼容版四元数与角度转换
# =========================
def quaternion_to_yaw(**kwargs) -> float:
    x = kwargs.get('x', kwargs.get('qx', 0.0))
    y = kwargs.get('y', kwargs.get('qy', 0.0))
    z = kwargs.get('z', kwargs.get('qz', 0.0))
    w = kwargs.get('w', kwargs.get('qw', 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def normalize_angle(angle: float) -> float:
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def mod2pi(theta: float) -> float:
    """Dubins 专用：将角度严格映射到 [0, 2π)"""
    return theta % (2.0 * math.pi)

# =========================
# 障碍物定义 (支持形状)
# =========================
@dataclass
class RectObstacle:
    x1: float; y1: float; x2: float; y2: float
    def __post_init__(self):
        self.x1, self.x2 = min(self.x1, self.x2), max(self.x1, self.x2)
        self.y1, self.y2 = min(self.y1, self.y2), max(self.y1, self.y2)

@dataclass
class CircleObstacle:
    x: float; y: float; r: float

@dataclass
class PolygonObstacle:
    points: List[Tuple[float, float]]
    def contains(self, px: float, py: float) -> bool:
        inside = False
        n = len(self.points)
        p1x, p1y = self.points[0]
        for i in range(n + 1):
            p2x, p2y = self.points[i % n]
            if py > min(p1y, p2y):
                if py <= max(p1y, p2y):
                    if px <= max(p1x, p2x):
                        if p1y != p2y:
                            xints = (py - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or px <= xints:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

ObstacleType = Union[RectObstacle, CircleObstacle, PolygonObstacle]

# =========================
# 自适应稀疏栅格地图
# =========================
class SparseGridMap:
    def __init__(self, x_min, x_max, y_min, y_max, resolution, obstacles, safety_margin):
        if resolution <= 0:
            raise ValueError("resolution 必须大于 0")

        self.resolution = float(resolution)
        self.occ = set()

        self.min_c = int(math.floor(x_min / self.resolution))
        self.max_c = int(math.ceil(x_max / self.resolution))
        self.min_r = int(math.floor(y_min / self.resolution))
        self.max_r = int(math.ceil(y_max / self.resolution))

        self._build_obstacles(obstacles)
        self._dilate_obstacles(safety_margin)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return int(math.floor(x / self.resolution)), int(math.floor(y / self.resolution))

    def cell_to_world(self, c: int, r: int) -> Tuple[float, float]:
        return c * self.resolution, r * self.resolution

    def _build_obstacles(self, obstacles):
        for obs in obstacles:
            if isinstance(obs, CircleObstacle):
                min_c, min_r = self.world_to_cell(obs.x - obs.r, obs.y - obs.r)
                max_c, max_r = self.world_to_cell(obs.x + obs.r, obs.y + obs.r)

                for c in range(min_c, max_c + 1):
                    for r in range(min_r, max_r + 1):
                        wx, wy = self.cell_to_world(c, r)
                        if math.hypot(wx - obs.x, wy - obs.y) <= obs.r:
                            self.occ.add((c, r))

            elif isinstance(obs, RectObstacle):
                min_c, min_r = self.world_to_cell(obs.x1, obs.y1)
                max_c, max_r = self.world_to_cell(obs.x2, obs.y2)

                for c in range(min_c, max_c + 1):
                    for r in range(min_r, max_r + 1):
                        self.occ.add((c, r))

            elif isinstance(obs, PolygonObstacle):
                if len(obs.points) < 3:
                    continue

                min_px = min(p[0] for p in obs.points)
                max_px = max(p[0] for p in obs.points)
                min_py = min(p[1] for p in obs.points)
                max_py = max(p[1] for p in obs.points)

                min_c, min_r = self.world_to_cell(min_px, min_py)
                max_c, max_r = self.world_to_cell(max_px, max_py)

                for c in range(min_c, max_c + 1):
                    for r in range(min_r, max_r + 1):
                        wx, wy = self.cell_to_world(c, r)
                        if obs.contains(wx, wy):
                            self.occ.add((c, r))

    def _dilate_obstacles(self, safety_margin):
        if safety_margin <= 0 or not self.occ:
            return

        margin_cells = int(math.ceil(safety_margin / self.resolution))
        original_occ = list(self.occ)

        for c, r in original_occ:
            for dc in range(-margin_cells, margin_cells + 1):
                for dr in range(-margin_cells, margin_cells + 1):
                    dist = math.hypot(dc, dr) * self.resolution
                    if dist <= safety_margin:
                        self.occ.add((c + dc, r + dr))

    def is_free_world(self, x: float, y: float) -> bool:
        c, r = self.world_to_cell(x, y)

        if not (self.min_c <= c <= self.max_c and self.min_r <= r <= self.max_r):
            return False

        return (c, r) not in self.occ


# =========================
# Dubins 曲线生成器 (纯解析数学)
# =========================
def dubins_shot(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, r, step_size):
    """计算起点到终点的完美数学连线"""
    dx, dy = goal_x - start_x, goal_y - start_y
    d = math.hypot(dx, dy) / r
    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(start_yaw - theta)
    beta = mod2pi(goal_yaw - theta)

    best_len = float('inf')
    best_path = None

    def LSL(a, b, d):
        p_sq = 2 + d**2 - 2*math.cos(a-b) + 2*d*(math.sin(a)-math.sin(b))
        if p_sq < 0: return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(math.cos(b)-math.cos(a), d+math.sin(a)-math.sin(b))
        return [mod2pi(-a+tmp), p, mod2pi(b-tmp)], ['L', 'S', 'L']

    def RSR(a, b, d):
        p_sq = 2 + d**2 - 2*math.cos(a-b) + 2*d*(math.sin(b)-math.sin(a))
        if p_sq < 0: return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(math.cos(a)-math.cos(b), d-math.sin(a)+math.sin(b))
        return [mod2pi(a-tmp), p, mod2pi(-b+tmp)], ['R', 'S', 'R']

    def RSL(a, b, d):
        p_sq = d**2 - 2 + 2*math.cos(a-b) - 2*d*(math.sin(a)+math.sin(b))
        if p_sq < 0: return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(math.cos(a)+math.cos(b), d-math.sin(a)-math.sin(b))
        return [mod2pi(a-tmp+math.atan2(2,p)), p, mod2pi(b-tmp+math.atan2(2,p))], ['R', 'S', 'L']

    def LSR(a, b, d):
        p_sq = d**2 - 2 + 2*math.cos(a-b) + 2*d*(math.sin(a)+math.sin(b))
        if p_sq < 0: return None
        p = math.sqrt(p_sq)
        tmp = math.atan2(-math.cos(a)-math.cos(b), d+math.sin(a)+math.sin(b))
        return [mod2pi(-a+tmp-math.atan2(-2,p)), p, mod2pi(-b+tmp-math.atan2(-2,p))], ['L', 'S', 'R']

    for solver in [LSL, RSR, RSL, LSR]:
        res = solver(alpha, beta, d)
        if res:
            lengths, modes = res
            total_len = sum(lengths)
            if total_len < best_len:
                best_len = total_len
                best_path = (lengths, modes)

    if not best_path: return None

    # 将数学长度转化为离散的实际坐标点
    lengths, modes = best_path
    curr_x, curr_y, curr_yaw = start_x, start_y, start_yaw
    points = []
    
    for length, mode in zip(lengths, modes):
        if length == 0: continue
        dist = length * r
        num_steps = int(math.ceil(dist / step_size))
        actual_step = dist / num_steps
        
        for _ in range(num_steps):
            if mode == 'S':
                curr_x += actual_step * math.cos(curr_yaw)
                curr_y += actual_step * math.sin(curr_yaw)
            elif mode == 'L':
                curr_yaw += actual_step / r
                curr_x += r * (math.sin(curr_yaw) - math.sin(curr_yaw - actual_step / r))
                curr_y -= r * (math.cos(curr_yaw) - math.cos(curr_yaw - actual_step / r))
            elif mode == 'R':
                curr_yaw -= actual_step / r
                curr_x -= r * (math.sin(curr_yaw) - math.sin(curr_yaw + actual_step / r))
                curr_y += r * (math.cos(curr_yaw) - math.cos(curr_yaw + actual_step / r))
            curr_yaw = normalize_angle(curr_yaw)
            points.append((curr_x, curr_y, curr_yaw))
            
    # 强制让最后一个点和绝对终点坐标百分百对齐，消除浮点数计算误差
    if points:
        points[-1] = (goal_x, goal_y, normalize_angle(goal_yaw))
    else:
        points.append((goal_x, goal_y, normalize_angle(goal_yaw)))

    return points



# =========================
# Hybrid A* 算法核心
# =========================
@dataclass
class Node:
    x: float; y: float; yaw: float; g: float; f: float
    parent: Optional['Node']

def hybrid_astar(grid: SparseGridMap, start: Tuple[float, float, float], goal: Tuple[float, float, float],
                 turning_radius: float, step_size: float) -> Optional[List[Node]]:
    
    yaw_res = math.radians(10)
    kappas = [0.0, 1.0 / turning_radius, -1.0 / turning_radius]
    
    start_node = Node(start[0], start[1], start[2], 0.0, 0.0, None)
    open_heap = [(0.0, 0, start_node)]
    closed_set = set()
    counter = 0
    max_iterations = 100000 
    
    while open_heap and counter < max_iterations:
        _, _, curr = heapq.heappop(open_heap)
        
        # 1. 触发 Dubins 命中测试：当距离终点足够近时，尝试直接用数学曲线连通
        dist_to_goal = math.hypot(curr.x - goal[0], curr.y - goal[1])
        # 触发距离：一般设为转弯半径的 4 到 6 倍
        if dist_to_goal < turning_radius * 5.0:
            dubins_points = dubins_shot(curr.x, curr.y, curr.yaw, goal[0], goal[1], goal[2], turning_radius, step_size)
            if dubins_points:
                # 检查这条 Dubins 连线是否会撞墙
                collision_free = True
                for px, py, _ in dubins_points:
                    if not grid.is_free_world(px, py):
                        collision_free = False
                        break
                
                # 如果完全没撞墙，直接返回拼接好的完美路径！
                if collision_free:
                    path = []
                    temp = curr
                    while temp:
                        path.append(temp)
                        temp = temp.parent
                    path = path[::-1]
                    
                    # 拼接 Dubins 曲线点
                    for px, py, pyaw in dubins_points:
                        path.append(Node(px, py, pyaw, 0, 0, None))
                    return path

        # 3D 闭表去重
        idx = (int(round(curr.x / grid.resolution)), 
               int(round(curr.y / grid.resolution)), 
               int(round(curr.yaw / yaw_res)))
        if idx in closed_set: continue
        closed_set.add(idx)
        
        # 物理节点扩展
        for kappa in kappas:
            if kappa == 0.0:
                nx, ny, nyaw = curr.x + step_size * math.cos(curr.yaw), curr.y + step_size * math.sin(curr.yaw), curr.yaw
            else:
                nx = curr.x + (1.0 / kappa) * (math.sin(curr.yaw + kappa * step_size) - math.sin(curr.yaw))
                ny = curr.y - (1.0 / kappa) * (math.cos(curr.yaw + kappa * step_size) - math.cos(curr.yaw))
                nyaw = normalize_angle(curr.yaw + kappa * step_size)
            
            if not grid.is_free_world(nx, ny): continue
            
            penalty = step_size * 0.2 if kappa != 0.0 else 0.0
            g = curr.g + step_size + penalty
            h_dist = math.hypot(nx - goal[0], ny - goal[1])
            h_yaw = abs(normalize_angle(nyaw - goal[2])) * turning_radius * 0.5
            
            counter += 1
            heapq.heappush(open_heap, (g + h_dist + h_yaw, counter, Node(nx, ny, nyaw, g, g + h_dist + h_yaw, curr)))
            
    return None


# =========================
# 运动段碰撞检测
# =========================
def simulate_motion(x: float, y: float, yaw: float, kappa: float, distance: float) -> Tuple[float, float, float]:
    if abs(kappa) < 1e-9:
        nx = x + distance * math.cos(yaw)
        ny = y + distance * math.sin(yaw)
        nyaw = yaw
    else:
        nx = x + (1.0 / kappa) * (math.sin(yaw + kappa * distance) - math.sin(yaw))
        ny = y - (1.0 / kappa) * (math.cos(yaw + kappa * distance) - math.cos(yaw))
        nyaw = normalize_angle(yaw + kappa * distance)

    return nx, ny, nyaw


def is_motion_collision_free(
    grid: SparseGridMap,
    x: float,
    y: float,
    yaw: float,
    kappa: float,
    step_size: float,
    check_step: float
) -> Tuple[bool, float, float, float]:

    if check_step <= 0:
        check_step = grid.resolution

    n = max(1, int(math.ceil(step_size / check_step)))

    nx, ny, nyaw = x, y, yaw

    for i in range(1, n + 1):
        dist = min(i * check_step, step_size)
        nx, ny, nyaw = simulate_motion(x, y, yaw, kappa, dist)

        if not grid.is_free_world(nx, ny):
            return False, nx, ny, nyaw

    return True, nx, ny, nyaw

# =========================
# 规划器对外 API
# =========================
class AdaptiveHybridPlanner:
    def plan(
        self,
        start: Dict[str, Any],
        goal: Dict[str, Any],
        obstacles: List[ObstacleType],
        safety_margin: float,
        turning_radius: float,
        point_spacing: float,
        resolution: Optional[float] = None
    ) -> Dict[str, Any]:

        if safety_margin < 0:
            return {"ok": False, "message": "safety_margin 不能小于 0", "path": []}

        if turning_radius <= 0:
            return {"ok": False, "message": "turning_radius 必须大于 0", "path": []}

        if point_spacing <= 0:
            return {"ok": False, "message": "point_spacing 必须大于 0", "path": []}

        if resolution is not None and resolution <= 0:
            return {"ok": False, "message": "resolution 必须大于 0", "path": []}

        sx, sy = start["point"]["x"], start["point"]["y"]
        syaw = normalize_angle(quaternion_to_yaw(**start["orientation"]))

        gx, gy = goal["point"]["x"], goal["point"]["y"]
        gyaw = normalize_angle(quaternion_to_yaw(**goal["orientation"]))

        if resolution is None:
            calc_res = min(safety_margin / 2.5 if safety_margin > 0 else point_spacing / 2.5,
                           point_spacing / 2.5)
            adaptive_res = max(0.1, min(calc_res, 1.0))
        else:
            adaptive_res = resolution

        all_x, all_y = [sx, gx], [sy, gy]

        for obs in obstacles:
            if isinstance(obs, RectObstacle):
                all_x.extend([obs.x1, obs.x2])
                all_y.extend([obs.y1, obs.y2])
            elif isinstance(obs, CircleObstacle):
                all_x.extend([obs.x - obs.r, obs.x + obs.r])
                all_y.extend([obs.y - obs.r, obs.y + obs.r])
            elif isinstance(obs, PolygonObstacle):
                for px, py in obs.points:
                    all_x.append(px)
                    all_y.append(py)

        pad = max(50.0, turning_radius * 5)

        try:
            grid = SparseGridMap(
                min(all_x) - pad,
                max(all_x) + pad,
                min(all_y) - pad,
                max(all_y) + pad,
                adaptive_res,
                obstacles,
                safety_margin
            )
        except Exception as e:
            return {"ok": False, "message": f"地图构建失败: {e}", "path": []}

        if not grid.is_free_world(sx, sy):
            return {"ok": False, "message": "起点处于障碍物或膨胀区内", "path": []}

        if not grid.is_free_world(gx, gy):
            return {"ok": False, "message": "终点处于障碍物或膨胀区内", "path": []}

        path_nodes = hybrid_astar(
            grid,
            (sx, sy, syaw),
            (gx, gy, gyaw),
            turning_radius,
            point_spacing
        )

        if not path_nodes:
            return {
                "ok": False,
                "message": "无法找到符合转弯半径的无碰撞路径",
                "path": []
            }

        output_path = []

        for i, node in enumerate(path_nodes):
            qx, qy, qz, qw = yaw_to_quaternion(node.yaw)
            output_path.append({
                "index": i,
                "point": {
                    "x": round(node.x, 3),
                    "y": round(node.y, 3)
                },
                "orientation": {
                    "x": round(qx, 4),
                    "y": round(qy, 4),
                    "z": round(qz, 4),
                    "w": round(qw, 4)
                }
            })

        return {
            "ok": True,
            "message": f"success. resolution={adaptive_res}",
            "path": output_path
        }
