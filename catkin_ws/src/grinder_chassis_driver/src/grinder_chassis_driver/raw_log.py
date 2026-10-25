"""Bounded diagnostic file writer. Disk latency never holds the serial bus."""
from collections import deque
import threading
import time


class RawLogWriter:
    def __init__(self, path, capacity=256):
        self.path = path
        self.capacity = capacity
        self.condition = threading.Condition()
        self.pending = deque()
        self.running = True
        self.dropped = 0
        self.errors = 0
        self.reported_drops = 0
        self.thread = threading.Thread(target=self._run, name="chassis_raw_log", daemon=True)
        self.thread.start()

    @staticmethod
    def line(message):
        now_ns = time.time_ns()
        seconds = now_ns // 1000000000
        stamp = "{}.{:03d}".format(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(seconds)),
                                  (now_ns // 1000000) % 1000)
        return "[{}] {}\n".format(stamp, message[:4096])

    def append(self, message):
        line = self.line(message)
        with self.condition:
            if not self.running:
                return
            if len(self.pending) >= self.capacity:
                self.pending.popleft()
                self.dropped += 1
            self.pending.append(line)
            self.condition.notify()

    def _write(self, lines):
        with open(self.path, "a", encoding="utf-8") as handle:
            handle.writelines(lines)

    def _run(self):
        while True:
            with self.condition:
                while self.running and not self.pending:
                    self.condition.wait()
                if not self.pending and not self.running:
                    return
                batch = [self.pending.popleft() for _ in range(min(64, len(self.pending)))]
                drops = self.dropped
            if drops > self.reported_drops:
                batch.insert(0, self.line("RAW_LOG_DROPPED total={}".format(drops)))
            try:
                self._write(batch)
                self.reported_drops = drops
            except Exception:
                with self.condition:
                    self.errors += 1
                    self.dropped += len(batch)

    def close(self, timeout=.2):
        with self.condition:
            self.running = False
            self.condition.notify_all()
        self.thread.join(timeout)
