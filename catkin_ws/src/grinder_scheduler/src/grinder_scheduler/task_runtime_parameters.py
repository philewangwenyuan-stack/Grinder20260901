"""Task-scoped setpoints. Never write persisted settings or publish enable commands."""

import math
import threading

from grinder_scheduler.models import SchedulerState


class TaskRuntimeParameters:
    def __init__(self):
        self._lock = threading.RLock()
        self._task_id = None
        self._disc_speed = None
        self._restore_speed = None

    @staticmethod
    def _require_task(node, task_id):
        if not task_id or task_id != node.task_config.task_id:
            raise ValueError("Task changed; refresh runtime settings")
        if node.state not in (SchedulerState.RUNNING, SchedulerState.PAUSED):
            raise ValueError("Runtime settings require a running or paused task")

    def disc_speed(self, task_id, default):
        with self._lock:
            return self._disc_speed if self._task_id == task_id and self._disc_speed is not None else default

    def execute(self, node, task_id, run_speed=None, disc_speed=None):
        with self._lock:
            self._require_task(node, task_id)
            if node._navigation_speed_reconfigure_parameter != "desired_linear_vel":
                raise ValueError("Task runtime speed currently requires the RPP controller")
            rpp = node._read_rpp_settings()
            speed_limit = min(float(node._max_chassis_run_speed), float(rpp["max_linear_vel"]))
            rpm_limit = int(node._task_runtime_max_disc_speed_rpm)
            if not math.isfinite(speed_limit) or speed_limit <= 0:
                raise ValueError("Navigation speed limit unavailable")
            if run_speed is not None and (not math.isfinite(run_speed) or not 0 < run_speed <= speed_limit):
                raise ValueError("Run speed must be positive and within the navigation limit")
            if disc_speed is not None and not 0 <= disc_speed <= rpm_limit:
                raise ValueError("Disc speed is outside the configured limit")
            if run_speed is not None:
                original = float(rpp["desired_linear_vel"])
                previous_restore = self._restore_speed
                # Record rollback before dynamic_reconfigure: timeout may follow partial application.
                if self._restore_speed is None:
                    self._restore_speed = original
                try:
                    applied = node._apply_rpp_settings({"desired_linear_vel": float(run_speed)})
                    effective = float(applied["desired_linear_vel"])
                    if not math.isfinite(effective) or not 0 < effective <= speed_limit:
                        raise RuntimeError("Controller returned an invalid effective speed")
                    self._require_task(node, task_id)
                except Exception:
                    if node.state in (SchedulerState.RUNNING, SchedulerState.PAUSED) and node.task_config.task_id == task_id:
                        node._apply_rpp_settings({"desired_linear_vel": original})
                        self._restore_speed = previous_restore
                    else:
                        self.clear(node)
                    raise
                rpp["desired_linear_vel"] = effective
            self._require_task(node, task_id)
            if run_speed is not None or disc_speed is not None:
                self._task_id = task_id
            if disc_speed is not None:
                self._disc_speed = int(disc_speed)
                # Cover setpoint changes must not replace transition-path RPM or re-enable the disc.
                if node.state == SchedulerState.RUNNING and node._disc_auto_cover_desired:
                    node._publish_task_runtime_disc_speed(int(disc_speed))
            return {
                "run_speed": float(rpp["desired_linear_vel"]),
                "disc_speed_rpm": self.disc_speed(task_id, node._chassis_settings["disc_speed_rpm"]),
                "max_run_speed": speed_limit,
                "max_disc_speed_rpm": rpm_limit,
            }

    def clear(self, node):
        with self._lock:
            self._task_id = None
            self._disc_speed = None
            if self._restore_speed is not None:
                node._apply_rpp_settings({"desired_linear_vel": self._restore_speed})
                self._restore_speed = None
