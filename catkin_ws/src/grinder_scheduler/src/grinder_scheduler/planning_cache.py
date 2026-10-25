"""Bounded immutable preprocessing bases, with private per-request derivatives."""
import copy
import hashlib
import json
import threading
from collections import OrderedDict


class PlanningCache:
    def __init__(self, entries=2, budget=128 * 1024 * 1024):
        self.entries, self.budget = max(0, entries), max(0, budget)
        self.lock = threading.Lock()
        self.items = OrderedDict()
        self.bytes = 0
        self.hits = self.misses = 0
        self.identity = None

    def prepare(self, grid, metadata, config, build):
        # Hash the *effective* obstacle/crop/erase grid, not just map_id.
        identity = json.dumps(metadata, sort_keys=True, default=str)
        key = (identity, grid.shape, grid.dtype.str,
               hashlib.sha256(grid.tobytes()).digest(),
               json.dumps(vars(config), sort_keys=True, default=str))
        with self.lock:
            if identity != self.identity:
                self.items.clear()
                self.bytes = 0
                self.identity = identity
            base = self.items.get(key)
            if base is not None:
                self.hits += 1
                self.items.move_to_end(key)
                return self._fork(base)
            self.misses += 1
        built = build()  # Never run the algorithm under the cache lock.
        size = built.raw_grid.nbytes + built.inflated_grid.nbytes
        with self.lock:
            if self.identity == identity and self.entries and size <= self.budget:
                old = self.items.pop(key, None)
                if old is not None:
                    self.bytes -= old.raw_grid.nbytes + old.inflated_grid.nbytes
                while self.items and (len(self.items) >= self.entries or self.bytes + size > self.budget):
                    _, removed = self.items.popitem(last=False)
                    self.bytes -= removed.raw_grid.nbytes + removed.inflated_grid.nbytes
                self.items[key] = self._fork(built)
                self.bytes += size
        return built

    @staticmethod
    def _fork(base):
        result = copy.copy(base)
        # Base arrays are readonly; all mutable derivative caches are private.
        result._cache = OrderedDict()
        result._cache_bytes = result._hits = result._misses = 0
        return result
