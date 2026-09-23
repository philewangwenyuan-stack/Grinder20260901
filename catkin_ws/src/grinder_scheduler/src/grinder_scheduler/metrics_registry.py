"""Small, thread-safe, in-process metrics for the device-side ROS nodes."""

import functools
import math
import threading
import time
from collections import deque


_BUCKETS_MS = (1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0, 5000.0)


class _Counter:
    def __init__(self, registry, name):
        self._registry = registry
        self._name = name

    def inc(self, amount=1):
        self._registry._inc(self._name, amount)


class _Gauge:
    def __init__(self, registry, name):
        self._registry = registry
        self._name = name

    def set(self, value):
        self._registry._set(self._name, value)


class _Histogram:
    def __init__(self, registry, name):
        self._registry = registry
        self._name = name

    def observe(self, value):
        self._registry._observe(self._name, value)


class _Timer:
    def __init__(self, histogram):
        self._histogram = histogram
        self._started = time.monotonic()
        self._stopped = False

    def stop(self):
        if not self._stopped:
            self._stopped = True
            self._histogram.observe((time.monotonic() - self._started) * 1000.0)

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        self.stop()


class MetricsRegistry:
    """Counters, gauges, fixed-bucket histograms and a bounded event ring."""

    def __init__(self, event_capacity=128):
        self._lock = threading.Lock()
        self._counters = {}
        self._gauges = {}
        self._histograms = {}
        self._events = deque(maxlen=max(1, int(event_capacity)))
        self._handles = {}

    def _handle(self, kind, name, constructor):
        key = (kind, str(name))
        with self._lock:
            if key not in self._handles:
                self._handles[key] = constructor(self, key[1])
            return self._handles[key]

    def counter(self, name):
        return self._handle("counter", name, _Counter)

    def gauge(self, name):
        return self._handle("gauge", name, _Gauge)

    def histogram(self, name):
        return self._handle("histogram", name, _Histogram)

    def timer(self, name):
        return _Timer(self.histogram(name))

    def timed(self, name):
        """Measure one complete call, including early returns and failures."""
        histogram = self.histogram(name)

        def decorate(function):
            @functools.wraps(function)
            def wrapped(*args, **kwargs):
                with _Timer(histogram):
                    return function(*args, **kwargs)

            return wrapped

        return decorate

    def _inc(self, name, amount):
        value = int(amount)
        if value < 0:
            raise ValueError("counter increment must be non-negative")
        with self._lock:
            self._counters[name] = self._counters.get(name, 0) + value

    def _set(self, name, value):
        number = float(value)
        if not math.isfinite(number):
            return
        with self._lock:
            self._gauges[name] = number

    def _observe(self, name, value):
        number = float(value)
        if not math.isfinite(number):
            return
        with self._lock:
            item = self._histograms.get(name)
            if item is None:
                item = {"count": 0, "sum": 0.0, "min": number, "max": number,
                        "buckets_ms": [0] * len(_BUCKETS_MS), "over_max_ms": 0}
                self._histograms[name] = item
            item["count"] += 1
            item["sum"] += number
            item["min"] = min(item["min"], number)
            item["max"] = max(item["max"], number)
            for index, ceiling in enumerate(_BUCKETS_MS):
                if number <= ceiling:
                    item["buckets_ms"][index] += 1
                    break
            else:
                item["over_max_ms"] += 1

    def event(self, name, **fields):
        # 事件环只存少量状态字段，不放地图数据、协议载荷或无限增长的标识。
        with self._lock:
            self._events.append({"time": time.time(), "name": str(name), "fields": fields})

    def snapshot(self):
        with self._lock:
            return {
                "counters": dict(self._counters),
                "gauges": dict(self._gauges),
                "histograms": {
                    name: dict(item, buckets_ms=list(item["buckets_ms"]))
                    for name, item in self._histograms.items()
                },
                "histogram_bucket_limits_ms": list(_BUCKETS_MS),
                "events": list(self._events),
            }
