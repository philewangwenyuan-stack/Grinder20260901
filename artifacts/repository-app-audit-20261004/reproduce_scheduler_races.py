"""Offline production-method probes; no ROS, network, serial or hardware."""
import ast
from enum import Enum
from pathlib import Path
import threading
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[2]
source = ROOT / 'catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py'
tree = ast.parse(source.read_text(encoding='utf-8-sig'))
methods = {node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}

class State(Enum):
    RUNNING = 'running'
    READY = 'ready'
    STOPPED = 'stopped'

class Message:
    def __init__(self, **values):
        self.__dict__.update(values)

events = []
class Publisher:
    def __init__(self, name): self.name = name
    def publish(self, value): events.append((self.name, getattr(value, 'data', None)))

ros = NS(loginfo=lambda *a: None, logwarn=lambda *a: None)
namespace = dict(rospy=ros, time=__import__('time'), SchedulerState=State,
                 Bool=Message, UInt16=Message, Int16=Message, WheelSpeedCommand=Message)
node_class = ast.ClassDef(name='ProbeNode', bases=[], keywords=[], body=[
    methods['_start_execution'], methods['_apply_emergency_stop'], methods['_tick_path_execution']
], decorator_list=[])
exec(compile(ast.fix_missing_locations(ast.Module(body=[node_class], type_ignores=[])), str(source), 'exec'), namespace)
node = namespace['ProbeNode']()
node._chassis_settings = {'run_speed': 0.1}
node._collision_imminent_lock = threading.Lock()
node._collision_imminent_active = False
node._tf_buffer = None
node.current_path = NS(points=[{}], path_version=7)
node.task_config = NS(active_work_region_id='', task_id='task')
node._exec_mode = 'test'
node.state = State.READY
node._exec_active = False
entered = threading.Event()
release = threading.Event()
def mode_status():
    entered.set()
    if not release.wait(5): raise RuntimeError('probe barrier timed out')
    return NS(state='READY', localization_ready=True)
node._super_lio_status_proxy = mode_status
for name in ['_ensure_super_lio_proxies', '_sync_task_regions_from_overlay', '_reset_disc_motion_guard',
             '_init_segment_execution_cursor', '_publish_path_to_navigation', '_publish_active_segment_plan',
             '_send_active_segment_goal', '_begin_task_execution_record']:
    setattr(node, name, lambda *a, **k: None)
node._sorted_work_region_ids = lambda: []
node._configured_disc_speed_rpm = lambda: 1000
node._set_chassis_enabled = lambda enabled: events.append(('chassis_enable', enabled))
node._set_cmd_vel_forward_runtime_active = lambda enabled, **k: events.append(('task_enable', enabled))
for name in ['work_mode_pub', 'disc_speed_pub', 'disc_enable_pub', 'light_pub',
             'chassis_emergency_stop_pub', 'wheel_cmd_pub']:
    setattr(node, name, Publisher(name))
start_result = []
thread = threading.Thread(target=lambda: start_result.append(node._start_execution()))
thread.start()
assert entered.wait(5)
node._apply_emergency_stop(True)
assert node.state == State.STOPPED
stop_boundary = len(events)
release.set()
thread.join(5)
assert not thread.is_alive()
after_stop = events[stop_boundary:]
assert ('disc_enable_pub', True) in after_stop and ('task_enable', True) in after_stop
assert node.state == State.RUNNING
print('start_blocked_then_estop: final_state={}, reenabled_after_stop={}'.format(node.state.name, after_stop))

# Execute the exact production cache-hit if branch. This is a branch-level probe,
# not a full PathPointPlanRequest end-to-end test.
handler = methods['_handle_path_plan_request_impl_core']
cache_branch = next(n for n in ast.walk(handler) if isinstance(n, ast.If)
                    and 'self._path_plan_request_cache_key' in ast.unparse(n.test))
node._path_plan_request_cache_key = 'same'
node._path_plan_request_cache_path_version = 7
namespace.update(self=node, request=NS(force_replan=False), plan_cache_key='same', planned=False)
exec(compile(ast.fix_missing_locations(ast.Module(body=[cache_branch], type_ignores=[])), str(source), 'exec'), namespace)
node._tick_path_execution()  # READY returns before any navigation/projection calls.
assert node.state == State.READY and node._exec_active
print('display_path_cache_hit: state={}, exec_active={}, path_tick_returns_early=True'.format(node.state.name, node._exec_active))
