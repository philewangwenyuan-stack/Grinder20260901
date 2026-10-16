"""ROS-free persistent software interlock and cancellation scopes (Python 3.8)."""
import os
import threading
from contextlib import contextmanager
from functools import wraps

from .indicator_emergency import EmergencyRecord


class MotionCancelled(RuntimeError):
    pass


class MotionInterlock:
    def __init__(self, path, legacy_path=None):
        self.lock = threading.RLock()
        self.local = threading.local()
        initial_path = os.path.abspath(os.path.expanduser(path))
        migrate = legacy_path and not os.path.exists(initial_path)
        self.record = EmergencyRecord(initial_path)
        self.latched = self.record.value
        self.generation = 0
        self.error = ""
        # First upgrade only: an existing emergency indication must not become
        # an unlocked motion controller. Later display clears stay independent.
        if migrate and EmergencyRecord(legacy_path).value:
            self.invalidate(emergency=True)
            self.persist()

    def invalidate(self, emergency=False):
        with self.lock:
            self.generation += 1
            if emergency:
                self.latched = True
                self.record.observe()
            return self.generation

    def persist(self):
        try:
            self.record.flush()
        except OSError as exc:
            with self.lock:
                self.latched = True
                self.error = "cannot persist motion interlock: {}".format(exc)
            return False
        return True

    def reset(self, generation):
        with self.lock:
            if generation != self.generation:
                return False
            revision = self.record.snapshot()[1]
        try:
            cleared = self.record.clear(expected_revision=revision)
        except OSError as exc:
            with self.lock:
                self.error = "cannot persist motion reset: {}".format(exc)
            return False
        with self.lock:
            if not cleared or generation != self.generation:
                # An ordinary stop may invalidate the reset after its durable
                # FALSE write. Restore the latch record before returning failure.
                if self.latched and not self.record.snapshot()[0]:
                    self.record.observe()
                failed = True
            else:
                self.latched = False
                self.generation += 1
                self.error = ""
                failed = False
        if failed:
            self.persist()
        return not failed

    def check(self):
        token = getattr(self.local, "token", self.generation)
        if self.latched or token != self.generation:
            raise MotionCancelled("emergency latched or execution superseded")

    @contextmanager
    def commit(self):
        with self.lock:
            self.check()
            yield

    @contextmanager
    def action(self, replace=False):
        previous = getattr(self.local, "token", None)
        with self.lock:
            self.check()
            if previous is None and replace:
                self.generation += 1
            self.local.token = self.generation if previous is None else previous
        try:
            yield
        finally:
            if previous is None:
                del self.local.token
            else:
                self.local.token = previous


def motion_action(cancel_result=None, replace=False):
    """Never hold the commit lock around localization/planning/service waits."""
    def decorate(method):
        @wraps(method)
        def run(self, *args, **kwargs):
            gate = self._motion
            try:
                with gate.action(replace):
                    result = method(self, *args, **kwargs)
                    with gate.commit():
                        return result
            except MotionCancelled:
                return cancel_result
        return run
    return decorate


class GuardedPublisher:
    """Serialize each short publish against trip; old scopes cannot publish."""
    def __init__(self, publisher, gate, safe):
        self.publisher, self.gate, self.safe = publisher, gate, safe

    def publish(self, message):
        with self.gate.lock:
            if not self.safe(message):
                self.gate.check()
            return self.publisher.publish(message)

    def __getattr__(self, name):
        return getattr(self.publisher, name)
