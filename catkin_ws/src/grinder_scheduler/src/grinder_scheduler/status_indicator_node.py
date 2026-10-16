"""Receive state only; never publishes wheel, disc or emergency commands."""
import json
import math
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from grinder_chassis_driver.msg import ChassisStatus
from grinder_scheduler.msg import SchedulerStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from grinder_scheduler.status_indicator import (
    IndicatorTransport, LightState, SimulatedPort, StateResolver, brightness_from_percent,
)


class StatusIndicatorNode:
    def __init__(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.resolver = StateResolver(
            float(rospy.get_param("~state_timeout_sec", 3.0)),
            ignore_chassis_status=rospy.get_param("~ignore_chassis_status", False))
        if self.resolver.ignore_chassis_status:
            rospy.logwarn("Indicator test mode: ignoring chassis status availability/connection")
        percent = rospy.get_param("~brightness_percent", None)
        # Preserve existing raw-scale configs when no percentage is supplied.
        self.brightness = (brightness_from_percent(percent) if percent is not None
                           else int(rospy.get_param("~brightness", 166)))
        if not 1 <= self.brightness <= 255:
            raise ValueError("brightness must be 1..255")
        self.interval = float(rospy.get_param("~send_interval_sec", 0.5))
        if not math.isfinite(self.interval) or not 0.1 <= self.interval <= 1.0:
            raise ValueError("send_interval_sec must be 0.1..1.0")
        self.backend = rospy.get_param("~backend", "mock")
        self.serial_number = str(rospy.get_param("~usb_serial_number", ""))
        self.device = str(rospy.get_param("~device", ""))
        if self.backend not in ("mock", "serial"):
            raise ValueError("backend must be mock or serial")
        if self.backend == "serial" and not (self.serial_number or self.device.startswith("/dev/serial/by-id/")):
            raise ValueError("set USB serial number or stable /dev/serial/by-id device")
        self.transport = IndicatorTransport(self._open_port)
        self._clear_pending = False
        self._clear_generation = 0
        self._clear_service_lock = threading.Lock()
        self.status_pub = rospy.Publisher("/indicator/state", String, queue_size=1, latch=True)
        self.diagnostics_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        self.subscribers = [
            rospy.Subscriber("/scheduler/status", SchedulerStatus, self._scheduler, queue_size=1),
            rospy.Subscriber("/chassis/status", ChassisStatus, self._chassis, queue_size=1),
            rospy.Subscriber("/scheduler/control_activity", String, self._activity, queue_size=1),
            rospy.Subscriber("/scheduler/indicator_emergency", Bool,
                             lambda msg: self._emergency("scheduler_emergency", msg), queue_size=1),
            rospy.Subscriber("/chassis/indicator_emergency", Bool,
                             lambda msg: self._emergency("chassis_emergency", msg), queue_size=1),
        ]
        self.clear_service = rospy.Service("/indicator/clear_emergency", Trigger, self._clear)
        self.worker = threading.Thread(target=self._run, name="status-indicator", daemon=True)
        rospy.on_shutdown(self._stop.set)
        self.worker.start()

    def _update(self, name, value):
        with self._lock:
            self.resolver.update(name, value, time.monotonic())

    def _scheduler(self, msg):
        self._update("scheduler", {"state": msg.state})

    def _chassis(self, msg):
        self._update("chassis", {"connected": bool(msg.connected), "disc_enabled": bool(msg.disc_enabled)})

    def _activity(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or any(type(value.get(k)) is not bool for k in ("automatic_active", "manual_active")):
                raise ValueError("missing boolean activity fields")
            if value.get("pause_reason", "") not in ("", "manual", "collision", "localization"):
                raise ValueError("invalid pause reason")
            self._update("activity", value)
        except (ValueError, TypeError):
            rospy.logwarn_throttle(5.0, "Invalid control activity; indicator will time out")

    def _emergency(self, name, msg):
        with self._lock:
            self.resolver.update(name, bool(msg.data), time.monotonic())
            if msg.data:
                self._clear_pending = False
                self._clear_generation += 1

    def _clear(self, _request):
        # Display reset only. Do not call chassis/clear_fault or enable services.
        if not self._clear_service_lock.acquire(False):
            return TriggerResponse(success=False, message="display reset already in progress")
        try:
            started = time.monotonic()
            with self._lock:
                clear_generation = self._clear_generation
                chassis_seen = "chassis_emergency" in self.resolver.samples or "chassis" in self.resolver.samples
            sources = ["/scheduler", "/chassis"]
            if self.resolver.ignore_chassis_status and not chassis_seen:
                try:
                    rospy.wait_for_service("/chassis/clear_emergency_indicator", timeout=0.1)
                except rospy.ROSException:
                    sources.remove("/chassis")
            for source in sources:
                service = source + "/clear_emergency_indicator"
                rospy.wait_for_service(service, timeout=1.0)
                result = rospy.ServiceProxy(service, Trigger)()
                if not result.success:
                    return TriggerResponse(success=False, message=result.message)
            # Require post-reset source publications; old FALSE snapshots are
            # insufficient to send an explicit controller reset.
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not self._stop.is_set():
                with self._lock:
                    if clear_generation != self._clear_generation:
                        return TriggerResponse(success=False, message="new emergency interrupted display reset")
                    records = [self.resolver.samples.get(source[1:] + "_emergency") for source in sources]
                    if all(r is not None and not r[0] and r[1] >= started for r in records):
                        self._clear_generation += 1
                        self._clear_pending = True
                        return TriggerResponse(success=True, message="display reset queued; motion unchanged; check /indicator/state for USB ACK")
                self._stop.wait(0.05)
            return TriggerResponse(success=False, message="emergency source reset not confirmed")
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return TriggerResponse(success=False, message=str(exc))
        finally:
            self._clear_service_lock.release()

    def _open_port(self):
        if self.backend == "mock":
            return SimulatedPort()
        import serial
        from serial.tools import list_ports
        device = self.device
        if self.serial_number:
            candidates = [p.device for p in list_ports.comports() if p.serial_number == self.serial_number]
            if len(candidates) != 1:
                raise IOError("USB serial number must match exactly one device")
            device = candidates[0]
        return serial.Serial(device, baudrate=115200, timeout=0.5, write_timeout=0.5)

    def _step(self):
        now = time.monotonic()
        with self._lock:
            state = self.resolver.resolve(now)
            clear = self._clear_pending and state != LightState.EMERGENCY
            generation = self._clear_generation
        acknowledged = self.transport.send(state, now, self.brightness, clear)
        with self._lock:
            if acknowledged and clear and generation == self._clear_generation:
                self._clear_pending = False
        self.status_pub.publish(String(data=json.dumps({
            "state": state.name, "backend": self.backend, "acknowledged": acknowledged,
            "ignore_chassis_status": self.resolver.ignore_chassis_status,
            "brightness": self.brightness,
            "brightness_percent": round(self.brightness * 100.0 / 255, 1),
            "clear_pending": self._clear_pending, "error": self.transport.last_error,
        })))
        diagnostic = DiagnosticStatus()
        diagnostic.name = "grinder/status_indicator"
        diagnostic.hardware_id = self.serial_number or self.device or "mock"
        diagnostic.level = DiagnosticStatus.OK if acknowledged else DiagnosticStatus.ERROR
        diagnostic.message = state.name if acknowledged else self.transport.last_error
        diagnostic.values = [KeyValue(key="backend", value=self.backend)]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [diagnostic]
        self.diagnostics_pub.publish(array)

    def _run(self):
        try:
            while not self._stop.is_set() and not rospy.is_shutdown():
                try:
                    self._step()
                except Exception as exc:
                    # Stop feeding the device: its wall-clock timeout turns red.
                    rospy.logerr_throttle(5.0, "Status indicator worker failed: %s", exc)
                    self.transport.close()
                self._stop.wait(self.interval)
        finally:
            self.transport.close()


def main():
    rospy.init_node("grinder_status_indicator")
    StatusIndicatorNode()
    rospy.spin()
