"""Durable elapsed-task clock, independent of deletable execution history (Python 3.8)."""
import datetime
import json
import os
import threading
import time


class WorkTimeStatistics:
    def __init__(self, path, legacy_path, wall=time.time, monotonic=time.monotonic,
                 timezone=None, checkpoint_seconds=5.0, start_writer=True):
        self.path = path
        self._wall, self._monotonic, self._timezone = wall, monotonic, timezone
        self._lock = threading.RLock()
        self._io_lock = threading.Lock()
        self._wake, self._closed = threading.Event(), threading.Event()
        self._checkpoint_seconds = checkpoint_seconds
        self._thread = None
        self._active_id = ""
        self._active_started = 0.0
        self._active_accounted_ms = 0
        self._total_ms, self._days, self._incomplete = 0, {}, 0
        self._load_error, self._save_error = "", ""
        self._persisted_signature = None
        try:
            if os.path.exists(path):
                self._load()
            else:
                self._import_history(legacy_path)
            # Initial import / crash recovery must be durable before history deletion
            # can be served. This runs at startup, before motion/control callbacks.
            self.flush()
        except Exception as exc:
            # Never silently reset a corrupt counter or reseed it from truncated history.
            self._load_error = str(exc)
        if start_writer and not self._load_error:
            self._thread = threading.Thread(target=self._writer, name="work-time-store", daemon=True)
            self._thread.start()

    @staticmethod
    def _nonnegative_int(value):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError("invalid nonnegative work-time counter")
        return value

    def _load(self):
        with open(self.path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        if payload.get("schema_version") != 1:
            raise ValueError("unsupported work-time schema")
        total = self._nonnegative_int(payload["total_work_ms"])
        days = payload["daily_work_ms"]
        if not isinstance(days, dict):
            raise ValueError("invalid daily work-time counters")
        for day, value in days.items():
            if datetime.date.fromisoformat(day).isoformat() != day:
                raise ValueError("invalid work-time date")
            self._nonnegative_int(value)
        if sum(days.values()) != total:
            raise ValueError("daily and lifetime work-time counters disagree")
        self._total_ms, self._days = total, dict(days)
        self._incomplete = self._nonnegative_int(payload.get("incomplete_record_count", 0))
        if payload.get("active_execution_id"):
            # A new process never resumes a previous process's clock. The unflushed
            # tail is unknown; preserve its checkpoint and exclude all downtime.
            self._incomplete += 1

    def _import_history(self, legacy_path):
        if not os.path.exists(legacy_path):
            return
        with open(legacy_path, "r", encoding="utf-8") as handle:
            records = json.load(handle).get("task_executions", [])
        if not isinstance(records, list):
            raise ValueError("invalid legacy execution history")
        seen = set()
        now = int(self._wall())
        for record in records:
            if not isinstance(record, dict):
                self._incomplete += 1
                continue
            key = record.get("execution_id") or (record.get("map_id"), record.get("task_id"), record.get("started_at"))
            if key in seen:
                continue
            seen.add(key)
            try:
                start, end = int(record.get("started_at", 0)), int(record.get("finished_at", 0))
            except (ValueError, TypeError):
                start = end = 0
            if start <= 0 or end < start or end > now:
                self._incomplete += 1
                continue
            self._add_interval(start * 1000, (end - start) * 1000)

    def _add_interval(self, start_ms, duration_ms):
        remaining = duration_ms
        while remaining > 0:
            local = datetime.datetime.fromtimestamp(start_ms / 1000.0, self._timezone)
            day = local.date()
            next_midnight = datetime.datetime.combine(day + datetime.timedelta(days=1),
                                                      datetime.time(), self._timezone)
            boundary_ms = int(next_midnight.timestamp() * 1000)
            amount = min(remaining, max(1, boundary_ms - start_ms))
            key = day.isoformat()
            self._days[key] = self._days.get(key, 0) + amount
            self._total_ms += amount
            start_ms += amount
            remaining -= amount

    def _advance(self):
        if not self._active_id:
            return
        elapsed_ms = max(self._active_accounted_ms, int((self._monotonic() - self._active_started) * 1000))
        delta = elapsed_ms - self._active_accounted_ms
        if delta:
            # Wall-clock corrections must not inflate or subtract elapsed time.
            # Calendar assignment follows the board's local clock at observation.
            self._add_interval(int(self._wall() * 1000) - delta, delta)
            self._active_accounted_ms = elapsed_ms

    def start(self, execution_id):
        with self._lock:
            if self._load_error or self._closed.is_set() or not execution_id or self._active_id == execution_id:
                return
            self._advance()
            self._active_id = str(execution_id)
            self._active_started = self._monotonic()
            self._active_accounted_ms = 0
        self._wake.set()

    def finish(self):
        with self._lock:
            self._advance()
            self._active_id = ""
        self._wake.set()

    def snapshot(self):
        with self._lock:
            self._advance()
            day = datetime.datetime.fromtimestamp(self._wall(), self._timezone).date().isoformat()
            return dict(schema_version=1, available=not bool(self._load_error or self._save_error),
                        total_work_seconds=self._total_ms // 1000,
                        today_work_seconds=self._days.get(day, 0) // 1000, day=day,
                        incomplete_record_count=self._incomplete,
                        message="work_time_unavailable" if self._load_error or self._save_error else "")

    def flush(self):
        # One disk writer at a time, never hold the accounting lock during I/O.
        with self._io_lock:
            with self._lock:
                if self._load_error:
                    return False
                self._advance()
                payload = dict(schema_version=1, total_work_ms=self._total_ms,
                               daily_work_ms=dict(self._days), incomplete_record_count=self._incomplete,
                               active_execution_id=self._active_id, saved_at=int(self._wall()))
                signature = (self._total_ms, self._active_id, self._incomplete)
                if signature == self._persisted_signature and not self._save_error:
                    return True
            try:
                directory = os.path.dirname(os.path.abspath(self.path))
                os.makedirs(directory, exist_ok=True)
                temporary = self.path + ".tmp"
                with open(temporary, "w", encoding="utf-8") as handle:
                    json.dump(payload, handle, ensure_ascii=False, separators=(",", ":"))
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(temporary, self.path)
                if hasattr(os, "O_DIRECTORY"):
                    descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
                    try:
                        os.fsync(descriptor)
                    finally:
                        os.close(descriptor)
                with self._lock:
                    self._save_error = ""
                    self._persisted_signature = signature
                return True
            except Exception as exc:
                with self._lock:
                    self._save_error = str(exc)
                return False

    def _writer(self):
        while not self._closed.is_set():
            self._wake.wait(self._checkpoint_seconds)
            self._wake.clear()
            self.flush()
        self.flush()

    def close(self):
        self.finish()
        self._closed.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        else:
            self.flush()
