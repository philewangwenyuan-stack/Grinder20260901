"""Periodic bounded JSON snapshots; no file I/O on request handling threads."""

import json
import os
import threading
import time


class MetricsExporter:
    def __init__(self, registry, directory, process_name, interval_sec=10.0, logwarn=None):
        self._registry = registry
        self._directory = os.path.abspath(os.path.expanduser(str(directory)))
        self._process_name = str(process_name)
        self._interval_sec = max(1.0, float(interval_sec))
        self._logwarn = logwarn
        self._stop_event = threading.Event()
        self._export_lock = threading.Lock()
        self._thread = None
        self.path = os.path.join(self._directory, "{}.json".format(self._process_name))

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="metrics_export", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None and threading.current_thread() is not self._thread:
            self._thread.join(timeout=2.0)
        try:
            self.export_once()
        except Exception as exc:
            if self._logwarn is not None:
                self._logwarn("Final metrics export failed: %s", exc)

    def _run(self):
        while not self._stop_event.wait(self._interval_sec):
            try:
                self.export_once()
            except Exception as exc:
                if self._logwarn is not None:
                    self._logwarn("Metrics export failed: %s", exc)

    def export_once(self):
        # 固定覆盖单个快照文件，避免设备长期运行后指标日志无限增长。
        with self._export_lock:
            os.makedirs(self._directory, exist_ok=True)
            snapshot = self._registry.snapshot()
            snapshot.update({"timestamp": time.time(), "process": self._process_name,
                             "pid": os.getpid()})
            temporary = self.path + ".tmp"
            try:
                with open(temporary, "w", encoding="utf-8") as handle:
                    json.dump(snapshot, handle, ensure_ascii=False, separators=(",", ":"))
                os.replace(temporary, self.path)
            finally:
                if os.path.exists(temporary):
                    os.remove(temporary)
        return self.path
