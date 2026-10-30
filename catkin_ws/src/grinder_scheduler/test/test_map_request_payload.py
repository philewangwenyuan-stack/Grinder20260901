"""Raw preview equivalence, snapshot ownership and cache invalidation, without ROS."""
import ast
import gc
import math
import os
from pathlib import Path
import secrets
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import zlib

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / 'catkin_ws/src/grinder_scheduler/src'
sys.path.insert(0, str(SOURCE))
sys.path.insert(0, str(ROOT / 'third_party/sl_linka/sdk/python'))
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
from sl_link.message_gen import sl_link_pb2 as pb
from grinder_scheduler import map_grid_utils as utils
from grinder_scheduler.metrics_registry import MetricsRegistry


def message(grid):
    return NS(info=NS(width=grid.shape[1], height=grid.shape[0], resolution=.05,
                      origin=NS(position=NS(x=-1., y=2.))),
              header=NS(frame_id='map', stamp=NS(secs=123, nsecs=456000000)),
              data=tuple(int(v) for v in grid.ravel()))


def legacy_preview(grid, edge=640):
    image = np.full((*grid.shape, 3), 180, dtype=np.uint8)
    image[grid == 0] = (245, 245, 245)
    image[grid >= 100] = (45, 45, 45)
    image = cv2.flip(image, 0)
    scale = min(1., edge / float(max(grid.shape)))
    size = (max(1, round(grid.shape[1] * scale)), max(1, round(grid.shape[0] * scale)))
    return cv2.resize(image, size, interpolation=cv2.INTER_AREA) if size != image.shape[1::-1] else image


class GridTest(unittest.TestCase):
    def test_bridge_callback_accepts_ndarray_without_truth_test(self):
        path = SOURCE/'grinder_scheduler/ros_data_bridge.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RosDataBridge')
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and
                    n.name in ('_map_callback', 'get_map_with_seq')]
        namespace = dict(occupancy_grid_array=utils.occupancy_grid_array,
                         rospy=NS(logwarn_throttle=lambda *args: None))
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), namespace)
        bridge = namespace['RosDataBridge']()
        bridge._lock, bridge._map_seq, bridge._map_msg = threading.Lock(), 0, None
        msg = message(np.zeros((3, 4), np.int8))
        msg.data = np.frombuffer(bytes(12), np.int8)
        bridge._map_callback(msg)
        self.assertEqual((msg, 1), bridge.get_map_with_seq())
        self.assertIsInstance(msg.data, np.ndarray)
        self.assertFalse(msg.data.flags.writeable)
        bad = message(np.zeros((3, 4), np.int8))
        for data in (None, np.zeros(11, np.int8), np.zeros(12, np.int16)):
            bad.data = data
            bridge._map_callback(bad)
            self.assertEqual((msg, 1), bridge.get_map_with_seq())

    def test_thresholds_direction_and_all_signed_grid_values(self):
        image = np.array([[0, 10, 11], [249, 250, 255]], dtype=np.uint8)
        old = np.full(image.shape, -1, dtype=np.int16)
        old[np.flipud(image) >= 250] = 0
        old[np.flipud(image) <= 10] = 100
        grid = utils.saved_image_to_grid(image)
        np.testing.assert_array_equal(old, grid)
        np.testing.assert_array_equal(legacy_preview(old)[:, :, 0], utils.saved_image_to_gray_preview(image))
        self.assertFalse(grid.flags.writeable)
        signed = np.arange(-128, 128, dtype=np.int16).astype(np.int8).reshape(16, 16)
        np.testing.assert_array_equal(legacy_preview(signed)[:, :, 0], utils.grid_to_gray_preview(signed))
        np.testing.assert_array_equal(utils.occupancy_grid_array(message(signed)), signed)
        with self.assertRaises(ValueError):
            utils.grid_to_gray_preview(old)

    def test_numpy_view_owns_receive_buffer_and_never_builds_a_sequence(self):
        original = np.arange(-128, 128, dtype=np.int16).astype(np.int8).reshape(16, 16)
        msg = message(original)
        buffer = original.tobytes()
        msg.data = np.frombuffer(buffer, dtype=np.int8)
        with patch.object(utils.np, 'fromiter', side_effect=AssertionError('must use received view')):
            grid = utils.occupancy_grid_array(msg)
        self.assertTrue(np.shares_memory(grid, msg.data))
        with self.assertRaises(ValueError): grid[0, 0] = 0
        del msg, buffer
        gc.collect()
        np.testing.assert_array_equal(grid, original)

    def test_mutable_input_is_copied_and_invalid_array_rejected(self):
        original = np.zeros((3, 4), dtype=np.int8)
        msg = message(original)
        msg.data = original.ravel()
        grid = utils.occupancy_grid_array(msg)
        original[:] = 100
        self.assertTrue(np.all(grid == 0))
        for data in (None, [], np.zeros(12, np.int16), np.zeros((4, 3), np.int8)):
            msg.data = data
            with self.assertRaises(ValueError): utils.occupancy_grid_array(msg)

    def test_saved_service_keeps_grid_after_cache_clear_and_live_updates(self):
        from grinder_scheduler.map_service import MapService
        grid = utils.saved_image_to_grid(np.array([[0, 255], [205, 0]], np.uint8))
        source = utils.MapGridSource(('saved', 1), 2, 2, .05, -2., 3., 'map', 123456)
        service = MapService()
        self.assertTrue(service.set_raw_grid(source, grid))
        self.assertTrue(np.shares_memory(service._raw_grid, grid))
        self.assertFalse(hasattr(service._raw_msg, 'data'))
        before = service.compose_map()
        for _ in range(4):
            live = MapService()
            live.set_raw_map(message(np.zeros((3, 4), np.int8)))
        np.testing.assert_array_equal(service.compose_map(), before)
        info = service.get_map_info()
        self.assertEqual((-2., 3., 123456000000), (info['origin_x'], info['origin_y'], info['stamp_ns']))

    def test_compose_and_erase_never_modify_shared_array(self):
        from grinder_scheduler.map_service import MapService
        grid = np.zeros((20, 30), np.int8)
        msg = message(grid)
        service = MapService()
        self.assertTrue(service.set_raw_map(msg))
        service._overlay[1, 2] = 100
        composed = service.compose_map()
        self.assertEqual(100, composed[1, 2])
        points = [{'x': -1., 'y': 2.}, {'x': -.5, 'y': 2.}, {'x': -.5, 'y': 2.5}]
        service._apply_erase_region_patch(points, .1)
        np.testing.assert_array_equal(service._raw_grid, grid)


class PayloadTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        self.yaml = self.folder/'map.yaml'
        self.pgm = self.folder/'map.pgm'
        self.yaml.write_text('image: map.pgm\nresolution: 0.05\norigin: [-1, 2, 0]\n')
        self.image = np.random.RandomState(13).choice([0, 10, 11, 249, 250, 255], (301, 799)).astype(np.uint8)
        cv2.imwrite(str(self.pgm), self.image)
        self.metrics = MetricsRegistry()
        self.builder = utils.MapPayloadBuilder(metrics=self.metrics)

    def read_yaml(self, path):
        return dict(line.strip().split(':', 1) for line in Path(path).read_text().splitlines())

    def source(self):
        raster = self.builder.saved.load('saved', str(self.yaml), str(self.pgm), self.read_yaml)
        return utils.MapGridSource(('saved', 'saved', raster.signature), self.image.shape[1], self.image.shape[0],
                                   .05, -1., 2., 'map', 100, raster=raster)

    def test_saved_fast_path_color_gray_resize_and_grid_bytes(self):
        source = self.source()
        old_grid = utils.saved_image_to_grid(self.image).astype(np.int16)
        with patch.object(self.builder.saved, 'grid', side_effect=AssertionError('PNG must not construct grid')):
            for gray in (False, True):
                data, width, height, sx, sy = self.builder.encode(source, 640, 'png', gray)
                image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                self.assertEqual(0 if gray else 2, data[25])  # PNG IHDR color type.
                np.testing.assert_array_equal(image, legacy_preview(old_grid))
                self.assertEqual((width, height), image.shape[1::-1])
                self.assertEqual(sx, width / source.width)
                self.assertEqual(sy, height / source.height)
        data = self.builder.encode(source, 640, 'grid')[0]
        self.assertEqual(old_grid.astype(np.int8).tobytes(), data)

    def test_live_preview_reuses_same_array_and_payload(self):
        msg = message(utils.saved_image_to_grid(self.image))
        source = utils.MapGridSource(('live', 1, id(msg)), msg.info.width, msg.info.height,
                                     .05, -1., 2., 'map', 100, message=msg)
        first = self.builder.encode(source, 640)
        self.assertIs(first, self.builder.encode(source, 640))
        image = cv2.imdecode(np.frombuffer(first[0], np.uint8), cv2.IMREAD_COLOR)
        np.testing.assert_array_equal(image, legacy_preview(np.asarray(msg.data).reshape(self.image.shape)))
        self.assertEqual(1, self.metrics.snapshot()['counters']['map_payload_cache_hit_total'])

    def test_file_changes_delete_and_read_race(self):
        first = self.source()
        self.assertIs(first.raster, self.source().raster)
        self.image[:] = 0
        cv2.imwrite(str(self.pgm), self.image)
        second = self.source()
        self.assertNotEqual(first.key, second.key)
        self.assertNotEqual(self.builder.encode(first, 640)[0], self.builder.encode(second, 640)[0])
        self.yaml.write_text('image: map.pgm\nresolution: 0.1\norigin: [-2, 3, 0]\n')
        third = self.source()
        self.assertEqual(.1, third.raster.resolution)
        self.assertEqual(-2., third.raster.origin_x)
        # Replace asset between imread and the post-read stat: old data must not escape.
        self.builder.clear()
        original = cv2.imread
        def read_and_replace(path, flags):
            result = original(path, flags)
            cv2.imwrite(str(self.pgm), np.full(self.image.shape, 255, np.uint8))
            # Force a distinct version even on filesystems whose automatic
            # inode timestamps only advance once per kernel tick.
            stat = self.pgm.stat()
            os.utime(str(self.pgm), ns=(stat.st_atime_ns, stat.st_mtime_ns + 1000000))
            return result
        with patch.object(utils.cv2, 'imread', side_effect=read_and_replace):
            with self.assertRaises(IOError): self.source()
        self.pgm.unlink()
        with self.assertRaises(FileNotFoundError): self.source()
        self.yaml.unlink()
        with self.assertRaises(FileNotFoundError): self.source()

    def test_lru_limits_failures_and_inflight_invalidation(self):
        cache = utils.ByteLRU(2, 10)
        entered, release = threading.Event(), threading.Event()
        def build():
            entered.set(); release.wait(2)
            return b'old', 3
        thread = threading.Thread(target=lambda: cache.get_or_build('a', build))
        thread.start(); self.assertTrue(entered.wait(1))
        cache.clear(); release.set(); thread.join(2)
        self.assertEqual(0, cache.retained_bytes)
        for key in range(4): cache.get_or_build(key, lambda: (b'xxxxxx', 6))
        self.assertEqual(6, cache.retained_bytes)
        def failed(): raise IOError('failure')
        with self.assertRaises(IOError): cache.get_or_build('bad', failed)
        self.assertEqual(b'good', cache.get_or_build('bad', lambda: (b'good', 4)))

    def test_concurrent_payload_consumers_share_one_build(self):
        source = self.source()
        original = utils.saved_image_to_gray_preview
        barrier = threading.Barrier(6)
        outputs = []
        def worker():
            barrier.wait()
            outputs.append(self.builder.encode(source, 640))
        with patch.object(utils, 'saved_image_to_gray_preview', wraps=original) as convert:
            threads = [threading.Thread(target=worker) for _ in range(6)]
            for t in threads: t.start()
            for t in threads: t.join(3)
        self.assertEqual(6, len(outputs))
        self.assertEqual(1, convert.call_count)
        self.assertTrue(all(item is outputs[0] for item in outputs))

    def test_atomic_replace_with_preserved_mtime_invalidates(self):
        first = self.source()
        old = self.pgm.stat()
        replacement = self.folder/'replacement.pgm'
        cv2.imwrite(str(replacement), np.full(self.image.shape, 255, np.uint8))
        os.utime(str(replacement), ns=(old.st_atime_ns, old.st_mtime_ns))
        os.replace(str(replacement), str(self.pgm))
        second = self.source()
        self.assertNotEqual(first.key, second.key)
        self.assertTrue(np.all(second.raster.image == 255))


def scheduler_fixture(map_service_class=None):
    """Execute actual protocol/snapshot methods with minimal ROS dependency stand-ins."""
    path = SOURCE/'grinder_scheduler/scheduler_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SchedulerNode')
    selected = {'_get_map_payload_builder', '_map_grid_source', '_load_saved_raw_grid',
                '_load_raw_grid_yaml', 'build_map_chunks', '_build_raw_map_snapshot', '_preview_meta',
                '_build_offline_map_service_for_map', '_export_runtime_map'}
    node.body = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in selected]
    registry = MetricsRegistry()
    namespace = dict(metrics=registry, DEFAULT_LIVE_MAP_ID='LIVE_MAP', MapPayloadBuilder=utils.MapPayloadBuilder, MapGridSource=utils.MapGridSource,
                     occupancy_grid_array=utils.occupancy_grid_array, np=np, cv2=cv2,
                     math=math, os=os, time=time, secrets=secrets, zlib=zlib,
                     rospy=NS(Time=NS(now=lambda: NS(secs=123, nsecs=0)), logwarn=lambda *a: None, loginfo=lambda *a: None),
                     MapService=map_service_class or (lambda: NS(load_local_state=lambda _: None, get_map_version=lambda: 7)))
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    instance = namespace['SchedulerNode']()
    instance.sl_link_server = NS(pb=pb)
    instance._live_map_id = 'LIVE_MAP'
    instance._is_live_map_id = lambda value: not value or value == 'LIVE_MAP'
    instance._map_request_encoding = 'png'
    instance._preview_max_edge_cap = 640
    instance._map_state_dir = lambda _: ''
    instance._apply_localization_covariance = lambda _: None
    instance._apply_alignment_yaw_to_response = lambda *a: None
    instance._sanitize_preview_edge = lambda edge, cap_edge: min(max(64, edge), cap_edge)
    instance._alignment_yaw_deg_for_sl_link_report = lambda _: 0.
    instance._app_rotation_deg_for_map_id = lambda _: 0.
    instance._rotation_alignment_delta_deg_for_map_id = lambda _: 0.
    return instance


class ProtocolTest(unittest.TestCase):
    def test_offline_factory_uses_metadata_and_shared_grid_without_list(self):
        from grinder_scheduler.map_service import MapService
        node = scheduler_fixture(map_service_class=MapService)
        node._draw_region_id_on_preview = node._draw_region_label_on_preview = True
        node._preview_rga_enabled, node._preview_rga_backend, node._preview_rga_available = False, 'auto', False
        with tempfile.TemporaryDirectory() as folder:
            image = np.array([[0, 255, 205], [255, 0, 205]], np.uint8)
            cv2.imwrite(str(Path(folder)/'map.pgm'), image)
            (Path(folder)/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [-1,2,0]\n')
            node._saved_raw_grid_paths_for_map = lambda _: (str(Path(folder)/'map.yaml'), str(Path(folder)/'map.pgm'))
            service = node._build_offline_map_service_for_map('saved')
            self.assertFalse(hasattr(service._raw_msg, 'data'))
            self.assertFalse(service._raw_grid.flags.writeable)
            node._get_map_payload_builder().clear()
            np.testing.assert_array_equal(service.compose_map(), utils.saved_image_to_grid(image))

    def test_export_numpy_map_preserves_saved_pgm_direction_and_thresholds(self):
        node = scheduler_fixture()
        grid = np.array([[-1, 0, 100], [1, 99, 100]], np.int8)
        msg = message(grid)
        msg.data = np.frombuffer(grid.tobytes(), np.int8)
        with tempfile.TemporaryDirectory() as folder:
            yaml, pgm = node._export_runtime_map(msg, folder, 'map.yaml', 'map.pgm',
                                                align_to_initial_yaw=False, crop_to_free_space=False)
            image = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
            expected = np.full(grid.shape, 205, np.uint8)
            expected[grid == 0] = 254
            expected[grid >= 100] = 0
            np.testing.assert_array_equal(image, np.flipud(expected))
            self.assertIn('origin: [-1.000000, 2.000000, 0.000000]', Path(yaml).read_text())

    def test_task_result_raw_snapshot_reuses_payload_and_keeps_geometry(self):
        node = scheduler_fixture()
        grid = np.array([[-1, 0, 100], [1, 99, 100]], np.int8)
        msg = message(grid)
        node.ros_data_bridge = NS(get_map_with_seq=lambda: (msg, 1))
        node.map_service = NS(get_map_version=lambda: 4)
        for gray in (False, True):
            node._map_preview_grayscale = gray
            result = node._build_raw_map_snapshot('LIVE_MAP', image_format='png')
            image = cv2.imdecode(np.frombuffer(result['image_data'], np.uint8), cv2.IMREAD_COLOR)
            np.testing.assert_array_equal(image, legacy_preview(grid))
            self.assertEqual((-1., 2., .05, 'map'),
                             (result['origin_x'], result['origin_y'], result['resolution'], result['frame_id']))
            self.assertEqual(4, result['version'])

    def test_cached_bytes_still_get_fresh_request_and_snapshot_ids(self):
        node = scheduler_fixture()
        msg = message(np.array([[-1, 0, 100], [1, 99, 100]], np.int8))
        node.ros_data_bridge = NS(get_map_with_seq=lambda: (msg, 1))
        node.map_service = NS(get_map_version=lambda: 4)
        snapshots = []
        for request_id in (11, 12):
            result, chunks = node.build_map_chunks(pb.MapRequest(map_id='LIVE_MAP', request_id=request_id).SerializeToString())
            result = pb.MapRequestResult.FromString(result)
            snapshots.append(result.snapshot_id)
            self.assertEqual(request_id, result.request_id)
            decoded = [pb.MapChunk.FromString(data) for data, _, _ in chunks]
            self.assertTrue(all(c.request_id == request_id and c.snapshot_id == result.snapshot_id for c in decoded))
            data = b''.join(c.data for c in decoded)
            self.assertEqual(result.payload_crc32, zlib.crc32(data) & 0xffffffff)
            self.assertEqual(result.payload_size, len(data))
            self.assertEqual((3, 2, .05), (decoded[0].width, decoded[0].height, round(decoded[0].resolution, 2)))
        self.assertNotEqual(*snapshots)
        node.ros_data_bridge = NS(get_map_with_seq=lambda: (None, 2))
        result, chunks = node.build_map_chunks(pb.MapRequest(request_id=13).SerializeToString())
        self.assertEqual(pb.MAP_REQUEST_STATUS_NOT_READY, pb.MapRequestResult.FromString(result).status)
        self.assertEqual([], chunks)

    def test_missing_saved_map_reports_not_found(self):
        node = scheduler_fixture()
        def missing(_): raise FileNotFoundError('deleted')
        node._load_saved_raw_grid = missing
        result, chunks = node.build_map_chunks(pb.MapRequest(map_id='deleted', request_id=1).SerializeToString())
        self.assertEqual(pb.MAP_REQUEST_STATUS_NOT_FOUND, pb.MapRequestResult.FromString(result).status)
        self.assertEqual([], chunks)


if __name__ == '__main__':
    unittest.main()
