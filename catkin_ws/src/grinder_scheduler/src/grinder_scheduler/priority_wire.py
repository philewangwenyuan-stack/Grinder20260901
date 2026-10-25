"""Serialize complete wire frames, giving waiting control ACKs priority."""
import threading
from contextlib import contextmanager


class PriorityWire:
    def __init__(self):
        self.condition = threading.Condition()
        self.busy = False
        self.controls = 0

    @contextmanager
    def hold(self, control=False):
        with self.condition:
            if control:
                self.controls += 1
            try:
                while self.busy or (not control and self.controls):
                    self.condition.wait()
                self.busy = True
            finally:
                if control:
                    self.controls -= 1
        try:
            yield
        finally:
            with self.condition:
                self.busy = False
                self.condition.notify_all()
