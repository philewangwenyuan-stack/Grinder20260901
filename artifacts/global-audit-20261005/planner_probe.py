"""Read production Python methods through AST; no ROS, hardware or native libs."""
import ast
import dataclasses
import heapq
import json
import math
import pathlib
import random
import types

ROOT = pathlib.Path(__file__).resolve().parents[2]

@dataclasses.dataclass
class Point:
    row: int
    col: int

class Grid:
    def __init__(self, cells):
        self.cells = cells
        self.shape = (len(cells), len(cells[0]))
    def __getitem__(self, index):
        return self.cells[index[0]][index[1]]

def method(path, class_name, name):
    tree = ast.parse((ROOT / path).read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    ns = dict(Point=Point, heapq=heapq, math=math)
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), ns)
    return ns[name]

def optimum(grid, start, goal):
    distances = {start: 0.0}
    pending = [(0.0, start)]
    while pending:
        cost, current = heapq.heappop(pending)
        if cost != distances[current]:
            continue
        if current == goal:
            return cost
        row, col = current
        for dr, dc in ((0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)):
            nr, nc = row+dr, col+dc
            if not (0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]) or grid[nr,nc]:
                continue
            if dr and dc and (grid[row+dr,col] or grid[row,col+dc]):
                continue
            next_cost = cost + (1.414 if dr and dc else 1.0)
            if next_cost < distances.get((nr,nc), math.inf):
                distances[nr,nc] = next_cost
                heapq.heappush(pending, (next_cost,(nr,nc)))
    return None

astar = method('third_party/path_planner/mst27/mst27.py', 'ObstacleAwareBoustrophedonPlanner', '_astar_search')
straight = method('catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py', 'PlannerAdapter', '_build_straight_connector')
collision = method('third_party/path_planner/mst27/mst27.py', 'MultiStagePathPlanner', '_path_collision_free')
result = {}
for seed in range(1000):
    rng = random.Random(seed)
    cells = [[int(rng.random() < .25) for _ in range(20)] for _ in range(20)]
    cells[0][0] = cells[-1][-1] = 0
    grid = Grid(cells)
    best = optimum(grid, (0,0), (19,19))
    if best is None:
        continue
    path = astar(None, grid, Point(0,0), Point(19,19))
    cost = sum(max(abs(a.row-b.row),abs(a.col-b.col)) + .414 * min(abs(a.row-b.row),abs(a.col-b.col)) for a,b in zip(path,path[1:]))
    if cost > best + .001:
        result['astar_nonoptimal'] = dict(seed=seed, returned_cost=round(cost,3), dijkstra_cost=round(best,3), excess_pct=round((cost/best-1)*100,2), grid=cells)
        break
wall = Grid([[int(col == 5) for col in range(11)] for row in range(11)])
connector = straight(None, types.SimpleNamespace(Point=Point), Point(5,1), Point(5,9))
result['straight_fallback'] = dict(points=[dataclasses.asdict(p) for p in connector], collision_free=collision(connector, wall))
base = [types.SimpleNamespace(path_type='r') for _ in range(3)]
assembled = []
adapter_tree = ast.parse((ROOT/'catkin_ws/src/grinder_scheduler/src/grinder_scheduler/planner_adapter.py').read_text(encoding='utf-8'))
lap_loop = next(n for n in ast.walk(adapter_tree) if isinstance(n,ast.For) and isinstance(n.target,ast.Name) and n.target.id == 'lap')
tagging = compile(ast.Module(body=lap_loop.body[:2],type_ignores=[]),'production_lap_tagging.py','exec')
for lap in range(1,4):
    lap_namespace = dict(base_points=base, lap=lap,region_id='r')
    exec(tagging,lap_namespace)
    assembled.extend(lap_namespace['lap_points'])
result['repeat_reference_semantics'] = dict(actual_tags=[p.path_type for p in assembled], distinct_objects=len(set(id(p) for p in assembled)), expected_points=9)
fingerprint = method('catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py', 'SchedulerNode', '_path_plan_request_cache_fingerprint')
config = types.SimpleNamespace(map_id='m',task_id='t',region_repeat_config={},work_regions=[],obstacle_regions=[],erase_regions=[],crop_region={},inflation_radius=.1,default_path_spacing=.7)
context = types.SimpleNamespace(task_config=config,_round_float=lambda value,digits=4: round(float(value),digits),_region_cache_fingerprint=lambda regions: (),_current_pose_cache_fingerprint=lambda: (0,0,0),_navigation_alignment_yaw=lambda: 0,_rotation_alignment_delta_deg_for_map_id=lambda map_id:0)
info = dict(map_version=1,width=100,height=100,resolution=.05,origin_x=0,origin_y=0,stamp_ns=1,seq=1)
key_before = fingerprint(context,info,[],{},{},'x',False)
config.inflation_radius = 1.5
config.default_path_spacing = 2.0
info['stamp_ns'] = 2
info['seq'] = 2
key_after = fingerprint(context,info,[],{},{},'x',False)
result['planning_cache_key'] = dict(unchanged_after_inflation_spacing_and_raw_map_stamp_change=key_before == key_after)
selection = method('catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py', 'SchedulerNode', '_effective_selected_work_region_ids')
regions = [dict(region_id='A'),dict(region_id='B')]
config.selected_work_region_ids = ['missing']
config.active_work_region_id = ''
all_fallback = selection(context,regions)
config.active_work_region_id = 'A'
active_fallback = selection(context,regions)
result['invalid_explicit_region_selection'] = dict(requested=['missing'],without_active_region=all_fallback,with_active_A=active_fallback)
print(json.dumps(result, ensure_ascii=False, indent=2))
