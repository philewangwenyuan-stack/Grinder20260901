"""Connection-scoped manual leases; ROS-free, using receiver monotonic time."""
import threading
import time


class ManualDelivery:
    def __init__(self):
        self.lock = threading.RLock()
        self.session = None
        self.epoch = 0

    def begin(self, session):
        with self.lock:
            self.epoch += 1
            self.session = session

    def capture(self, session, received_at, timeout, stop=False):
        with self.lock:
            if session is not self.session:
                return (session, -1, 0.0)
            if stop:
                self.epoch += 1
            return (session, self.epoch, float('inf') if stop else received_at + timeout)

    def invalidate(self, session=None):
        with self.lock:
            if session is None or session is self.session:
                self.epoch += 1
                return True
            return False

    def valid(self, ticket):
        with self.lock:
            return (ticket[0] is self.session and ticket[1] == self.epoch
                    and time.monotonic() <= ticket[2])


def is_manual_stop(command, pb):
    if command.speed_ratio <= 0:
        return True
    if abs(command.remote_x) > 1e-3 or abs(command.remote_y) > 1e-3:
        return False
    motions = ('FORWARD', 'BACKWARD', 'FORWARD_LEFT', 'FORWARD_RIGHT',
               'BACKWARD_LEFT', 'BACKWARD_RIGHT')
    return command.motion not in {getattr(pb, 'MANUAL_MOTION_' + m) for m in motions}
