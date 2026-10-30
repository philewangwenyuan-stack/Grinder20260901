"""Immutable raw-map snapshots and bounded caches; no ROS dependencies."""
import math
import os
import re
import threading
from collections import OrderedDict
from concurrent.futures import Future
from contextlib import nullcontext
from dataclasses import dataclass

import cv2
import numpy as np


SAVED_IMAGE_PREVIEW_LUT = np.full(256, 180, dtype=np.uint8)
SAVED_IMAGE_PREVIEW_LUT[:11] = 45
SAVED_IMAGE_PREVIEW_LUT[250:] = 245
SAVED_IMAGE_GRID_LUT = np.full(256, -1, dtype=np.int8)
SAVED_IMAGE_GRID_LUT[:11] = 100
SAVED_IMAGE_GRID_LUT[250:] = 0
GRID_PREVIEW_LUT = np.full(256, 180, dtype=np.uint8)
GRID_PREVIEW_LUT[0] = 245
GRID_PREVIEW_LUT[100:128] = 45
for _lut in (SAVED_IMAGE_PREVIEW_LUT, SAVED_IMAGE_GRID_LUT, GRID_PREVIEW_LUT):
    _lut.flags.writeable = False


def _timer(metrics, name):
    return metrics.timer(name) if metrics is not None else nullcontext()


class ByteLRU:
    """Coalesce concurrent builds without holding the cache lock during work."""
    def __init__(self, max_entries, max_bytes):
        self.max_entries = max_entries
        self.max_bytes = max_bytes
        self._entries = OrderedDict()
        self._pending = {}
        self._lock = threading.Lock()
        self._bytes = 0
        self._generation = 0

    @property
    def retained_bytes(self):
        with self._lock:
            return self._bytes

    def clear(self):
        with self._lock:
            self._generation += 1
            self._entries.clear()
            self._bytes = 0

    def get_or_build(self, key, build, metrics=None, metric_prefix=None):
        with self._lock:
            entry = self._entries.get(key)
            if entry is not None:
                self._entries.move_to_end(key)
                future, leader = None, False
            else:
                future = self._pending.get(key)
                leader = future is None
                if leader:
                    future = Future()
                    self._pending[key] = future
                generation = self._generation
        if metrics is not None and metric_prefix:
            suffix = "hit_total" if entry is not None else "miss_total"
            metrics.counter(metric_prefix + "_" + suffix).inc()
        if entry is not None:
            return entry[0]
        if not leader:
            return future.result()
        try:
            value, size = build()
            with self._lock:
                if generation == self._generation and 0 <= size <= self.max_bytes:
                    self._entries[key] = (value, size)
                    self._bytes += size
                    while len(self._entries) > self.max_entries or self._bytes > self.max_bytes:
                        _, (_, removed) = self._entries.popitem(last=False)
                        self._bytes -= removed
            future.set_result(value)
            return value
        except BaseException as error:
            future.set_exception(error)
            raise
        finally:
            with self._lock:
                self._pending.pop(key, None)


def occupancy_grid_array(msg, raw_grid=None):
    """Return an immutable 2D grid; numpy_msg byte-backed data requires no copy.

    Ordinary flat ROS sequences remain accepted at the compatibility boundary.
    Their conversion is owned by the caller, never retained in an identity cache.
    """
    width, height = int(msg.info.width), int(msg.info.height)
    data = getattr(msg, "data", None) if raw_grid is None else raw_grid
    if width <= 0 or height <= 0 or data is None:
        raise ValueError("incomplete occupancy grid")
    if isinstance(data, np.ndarray):
        if data.dtype != np.int8 or data.ndim not in (1, 2):
            raise ValueError("occupancy grid must be a flat or 2D int8 array")
        if data.size != width * height or (data.ndim == 2 and data.shape != (height, width)):
            raise ValueError("incomplete occupancy grid")
        # Copy mutable external input once so later edits cannot alter a snapshot.
        grid = data if data.flags.c_contiguous and not data.flags.writeable else data.copy(order="C")
        grid = grid.reshape(height, width)
    else:
        if len(data) != width * height:
            raise ValueError("incomplete occupancy grid")
        grid = np.fromiter(data, dtype=np.int8, count=width * height).reshape(height, width)
    grid.flags.writeable = False
    return grid


def saved_image_to_gray_preview(image):
    return cv2.LUT(image, SAVED_IMAGE_PREVIEW_LUT)


def saved_image_to_grid(image):
    grid = np.ascontiguousarray(SAVED_IMAGE_GRID_LUT[np.flipud(image)])
    grid.flags.writeable = False
    return grid


def grid_to_gray_preview(grid):
    if grid.dtype != np.int8 or grid.ndim != 2:
        raise ValueError("preview expects a two-dimensional int8 occupancy grid")
    return cv2.flip(GRID_PREVIEW_LUT[grid.view(np.uint8)], 0)


def file_signature(path):
    stat = os.stat(path)
    return (os.path.realpath(path), stat.st_dev, stat.st_ino,
            stat.st_mtime_ns, stat.st_ctime_ns, stat.st_size)


@dataclass(frozen=True)
class SavedRaster:
    image: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    signature: tuple


class SavedMapCache:
    def __init__(self, metrics=None):
        self.images = ByteLRU(4, 32 * 1024 * 1024)
        self.grids = ByteLRU(4, 32 * 1024 * 1024)
        self.metrics = metrics

    def clear(self):
        self.images.clear()
        self.grids.clear()

    def load(self, map_id, yaml_path, fallback_image, read_yaml):
        # Verify both files even on cache hits; deleted/updated assets never use stale bytes.
        for _ in range(3):
            yaml_sig = file_signature(yaml_path)
            meta = read_yaml(yaml_path)
            image_name = str(meta.get("image", "") or "").strip().strip("\"'")
            image_path = image_name or fallback_image
            if image_name and not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(yaml_path), image_path)
            image_sig = file_signature(image_path)
            signature = (yaml_sig, image_sig)

            def build():
                with _timer(self.metrics, "map_saved_file_read_ms"):
                    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
                if image is None:
                    raise IOError("failed to read saved raw grid image: " + image_path)
                resolution = float(meta.get("resolution", 0.05) or 0.05)
                origin = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", str(meta.get("origin", "0 0 0")))
                x = float(origin[0]) if origin else 0.0
                y = float(origin[1]) if len(origin) > 1 else 0.0
                if not all(math.isfinite(v) for v in (resolution, x, y)) or resolution <= 0:
                    raise ValueError("invalid saved map geometry")
                image.flags.writeable = False
                return SavedRaster(image, resolution, x, y, signature), image.nbytes

            raster = self.images.get_or_build((map_id, signature), build,
                                             self.metrics, "map_saved_cache")
            if (file_signature(yaml_path), file_signature(image_path)) == signature:
                return raster
        raise IOError("saved map changed while reading; retry")

    def grid(self, raster):
        def build():
            with _timer(self.metrics, "map_saved_gray_to_grid_ms"):
                grid = saved_image_to_grid(raster.image)
            return grid, grid.nbytes
        return self.grids.get_or_build(raster.signature, build)


@dataclass(frozen=True)
class MapGridSource:
    key: tuple
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str
    stamp_ms: int
    raster: object = None
    message: object = None
    grid_array: object = None


class MapPayloadBuilder:
    def __init__(self, saved=None, metrics=None):
        self.saved = saved or SavedMapCache(metrics)
        self.metrics = metrics
        self.payloads = ByteLRU(8, 32 * 1024 * 1024)

    def clear(self):
        self.saved.clear()
        self.payloads.clear()

    def grid(self, source):
        if source.grid_array is not None:
            return source.grid_array
        return (self.saved.grid(source.raster) if source.raster is not None
                else occupancy_grid_array(source.message))

    def encode(self, source, max_edge, image_format="png", grayscale=False):
        key = (source.key, int(max_edge), image_format, bool(grayscale))
        def build():
            if image_format == "grid":
                with _timer(self.metrics, "map_grid_encode_ms"):
                    data = self.grid(source).tobytes()
                value = (data, source.width, source.height, 1.0, 1.0)
            else:
                grid = self.grid(source) if source.raster is None else None
                with _timer(self.metrics, "map_colorize_ms"):
                    image = (saved_image_to_gray_preview(source.raster.image)
                             if source.raster is not None
                             else grid_to_gray_preview(grid))
                scale = min(1.0, max(64.0, float(max_edge)) / max(source.width, source.height))
                # Match SchedulerNode._preview_meta exactly.
                width = max(1, int(round(source.width * scale)))
                height = max(1, int(round(source.height * scale)))
                if not grayscale:
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                if (width, height) != (source.width, source.height):
                    with _timer(self.metrics, "map_resize_ms"):
                        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
                with _timer(self.metrics, "map_{}_encode_ms".format(image_format)):
                    okay, encoded = cv2.imencode("." + image_format, image)
                if not okay:
                    raise IOError("raw map image encoding failed")
                data = encoded.tobytes()
                value = (data, width, height, width / float(source.width), height / float(source.height))
            return value, len(data)
        return self.payloads.get_or_build(key, build, self.metrics, "map_payload_cache")
