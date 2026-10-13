"""Display-only emergency record. Never gates motion or resets a chassis fault."""
import json
import os
import threading


class EmergencyRecord:
    def __init__(self, path):
        self.path = os.path.abspath(os.path.expanduser(path))
        self._lock = threading.Lock()
        self._io_lock = threading.Lock()
        self.value = False
        self.revision = 0
        self.saved_revision = 0
        try:
            with open(self.path, encoding="utf-8") as source:
                data = json.load(source)
            if not isinstance(data, dict) or type(data.get("emergency_observed")) is not bool:
                raise ValueError("invalid emergency indicator record")
            self.value = data["emergency_observed"]
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError):
            # An unreadable record must not silently erase a previous emergency.
            self.value = True
            self.revision = 1

    def observe(self):
        # No disk I/O on the emergency/stop path.
        with self._lock:
            self.value = True
            self.revision += 1

    def snapshot(self):
        with self._lock:
            return self.value, self.revision

    def flush(self):
        with self._io_lock:
            value, revision = self.snapshot()
            if revision == self.saved_revision:
                return
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            temporary = self.path + ".tmp"
            with open(temporary, "w", encoding="utf-8") as target:
                json.dump({"emergency_observed": value}, target)
                target.flush()
                os.fsync(target.fileno())
            os.replace(temporary, self.path)
            self.saved_revision = revision

    def clear(self):
        with self._lock:
            self.value = False
            self.revision += 1
            revision = self.revision
        try:
            self.flush()
        except OSError:
            self.observe()
            raise
        value, current_revision = self.snapshot()
        return not value and current_revision == revision


class EmergencyIndicatorSource:
    """Publish promptly; persist and heartbeat on a separate wall-clock worker."""
    def __init__(self, namespace, default_path):
        import rospy
        from std_msgs.msg import Bool
        from std_srvs.srv import Trigger, TriggerResponse

        self._rospy = rospy
        self._Bool = Bool
        self._Response = TriggerResponse
        self.record = EmergencyRecord(rospy.get_param("~indicator_emergency_file", default_path))
        self._publish_lock = threading.Lock()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self.publisher = rospy.Publisher(namespace + "/indicator_emergency", Bool, queue_size=1, latch=True)
        self.service = rospy.Service(namespace + "/clear_emergency_indicator", Trigger, self._clear)
        self.publish()
        self.worker = threading.Thread(target=self._run, name="emergency-indicator", daemon=True)
        self.worker.start()
        rospy.on_shutdown(self._stop.set)

    def publish(self):
        with self._publish_lock:
            self.publisher.publish(self._Bool(data=self.record.snapshot()[0]))

    def observe(self):
        self.record.observe()
        try:
            self.publish()
        except Exception as exc:
            self._rospy.logerr("Cannot publish emergency indicator: %s", exc)
        self._wake.set()

    def _clear(self, _request):
        try:
            success = self.record.clear()
            message = "display record cleared; motion unchanged" if success else "new emergency observed during clear"
        except OSError as exc:
            success, message = False, "cannot persist display reset: {}".format(exc)
        self.publish()
        self._wake.set()
        return self._Response(success=success, message=message)

    def _run(self):
        while not self._stop.is_set():
            try:
                self.record.flush()
                self.publish()
            except Exception as exc:
                self._rospy.logerr_throttle(5.0, "Emergency indicator persistence/publication failed: %s", exc)
            self._wake.wait(0.5)
            self._wake.clear()
