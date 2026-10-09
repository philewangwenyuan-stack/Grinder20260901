"""Read current driver methods via AST; use fake transport, never ROS/hardware."""
import ast
import importlib.util
import pathlib
import threading
import time
from types import SimpleNamespace

ROOT = pathlib.Path(__file__).resolve().parents[2]
PKG = ROOT / "catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver"
spec = importlib.util.spec_from_file_location("audit_registers", PKG / "register_map.py")
registers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(registers)
tree = ast.parse((PKG / "chassis_driver_node.py").read_text(encoding="utf-8"))
methods = {"_write_all_outputs", "_write_motion_registers", "_write_single_output",
           "_issue_safe_stop", "_apply_ramp_limit", "_enforce_command_timeout"}
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
ns = vars(registers).copy()
ns.update(time=time, ModbusTransportError=RuntimeError,
          rospy=SimpleNamespace(logwarn_throttle=lambda *a: None,
                                Time=SimpleNamespace(now=lambda: 10)))
exec(compile(ast.Module(body=[cls], type_ignores=[]), str(PKG / "chassis_driver_node.py"), "exec"), ns)


class Transport:
    def __init__(self):
        self.writes = []

    def write_register_block(self, address, values):
        self.writes.append((address, tuple(values)))

    def write_uint16(self, address, value):
        self.write_register_block(address, [value])

    write_int16 = write_uint16

    def log_event(self, *args):
        pass


def node():
    obj = ns[cls.name]()
    obj._lock = threading.RLock()
    obj.command = registers.ChassisCommand()
    obj.enabled = True
    obj.disc_lift_supported = False
    obj._last_written_block = None
    obj._last_written_registers = {}
    obj.last_sent_left_wheel_speed = 0
    obj.last_sent_right_wheel_speed = 0
    obj.enable_cmd_ramp_limit = False
    obj.transport = Transport()
    obj._mark_connected = lambda: None
    obj._handle_transport_error = lambda exc: (_ for _ in ()).throw(exc)
    return obj


obj = node()
obj._write_all_outputs()
obj.command.left_wheel_speed = obj.command.right_wheel_speed = 100
obj._write_motion_registers()
before = len(obj.transport.writes)
obj._issue_safe_stop()
print("safe_stop_after_motion: additional_frames={}, last_sent_wheels=({}, {})".format(
    len(obj.transport.writes) - before,
    obj.last_sent_left_wheel_speed, obj.last_sent_right_wheel_speed))
assert len(obj.transport.writes) == before
assert obj.last_sent_left_wheel_speed == 100

obj = node()
obj._write_all_outputs()
obj.command.disc_enable = 1
obj._write_single_output(registers.REGISTER_DISC_ENABLE, 1, False)
before = len(obj.transport.writes)
obj._issue_safe_stop()
print("safe_stop_after_disc_enable: additional_frames={}".format(len(obj.transport.writes) - before))
assert len(obj.transport.writes) == before

obj = node()
obj.command.left_wheel_speed = obj.command.right_wheel_speed = 100
obj._write_motion_registers()
obj.enable_cmd_ramp_limit = True
obj.max_cmd_step_rpm = 20
obj.stop_on_timeout = True
obj.command_timeout = 1

class OldTime:
    def __rsub__(self, other):
        return SimpleNamespace(to_sec=lambda: 10)

obj.last_command_time = OldTime()
obj._enforce_command_timeout()
obj._enforce_command_timeout()
print("timeout_with_ramp: last_sent_wheels=({}, {})".format(
    obj.last_sent_left_wheel_speed, obj.last_sent_right_wheel_speed))
assert obj.last_sent_left_wheel_speed == 80
print("All three suspected defects reproduced with fake transport.")
