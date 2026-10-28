"""ROS-free admission for one tablet and its independent data/control sockets."""
import secrets
import threading
import time


class ChannelSessions:
    def __init__(self, timeout=3.0, clock=time.monotonic):
        self.lock = threading.RLock()
        self.timeout = float(timeout)
        self.clock = clock
        self.token = None
        self.legacy = False
        self.channels = {}
        self.seen = {}

    def _expire_locked(self):
        now = self.clock()
        expired = [peer for role, peer in self.channels.items()
                   if now - self.seen[role] >= self.timeout]
        for peer in expired:
            self._detach_locked(peer)
        return expired

    def expire(self):
        with self.lock:
            return self._expire_locked()

    def _detach_locked(self, peer):
        for role, current in list(self.channels.items()):
            if current is peer:
                del self.channels[role]
                self.seen.pop(role, None)
        if not self.channels:
            self.token = None
            self.legacy = False

    def detach(self, peer):
        with self.lock:
            self._detach_locked(peer)

    def attach(self, role, peer, token="", legacy=False):
        """Return (accepted, token, retired). Never retire a live foreign owner."""
        with self.lock:
            retired = self._expire_locked()
            if role not in ("data", "control"):
                return False, "", retired
            if self.token is None:
                if role != "data" or token:
                    return False, "", retired
                self.token = secrets.token_hex(24)
                self.legacy = bool(legacy)
            elif self.channels.get(role) is not peer:
                if not token or not secrets.compare_digest(token, self.token) or self.legacy:
                    return False, "", retired
            if role == "control" and self.legacy:
                return False, "", retired
            previous = self.channels.get(role)
            if previous is not None and previous is not peer:
                retired.append(previous)
            self.channels[role] = peer
            self.seen[role] = self.clock()
            return True, self.token, retired

    def valid(self, role, peer):
        with self.lock:
            return (self.channels.get(role) is peer
                    and self.clock() - self.seen[role] < self.timeout)

    def touch(self, role, peer):
        with self.lock:
            if not self.valid(role, peer):
                return False
            self.seen[role] = self.clock()
            return True

    def control_ready(self):
        with self.lock:
            peer = self.channels.get("control")
            return peer is not None and self.valid("control", peer)

    def release(self, role, peer):
        with self.lock:
            if not self.valid(role, peer):
                return []
            retired = list(self.channels.values())
            self.channels.clear()
            self.seen.clear()
            self.token = None
            self.legacy = False
            return retired


class BulkBudget:
    """One token bucket shared by every bulk response; never holds a wire lock."""
    def __init__(self, rate=1048576, manual_rate=262144, burst=8192):
        self.rate = max(1, int(rate))
        self.manual_rate = max(1, int(manual_rate))
        self.burst = max(4096, int(burst))
        self.lock = threading.Lock()
        self.tokens = float(self.burst)
        self.updated = time.monotonic()
        self.manual_until = 0.0

    def manual(self, duration=0.5):
        with self.lock:
            self.manual_until = time.monotonic() + duration

    def consume(self, size, valid):
        remaining = int(size)
        while remaining > 0:
            if not valid():
                raise ConnectionError("Bulk transfer session ended")
            with self.lock:
                now = time.monotonic()
                rate = self.manual_rate if now < self.manual_until else self.rate
                self.tokens = min(self.burst, self.tokens + (now - self.updated) * rate)
                self.updated = now
                used = min(remaining, int(self.tokens))
                self.tokens -= used
                remaining -= used
            if remaining:
                time.sleep(min(0.02, max(0.001, remaining / float(rate))))
