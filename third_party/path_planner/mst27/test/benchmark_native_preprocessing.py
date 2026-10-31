"""Isolated offline preprocessing benchmark; never imports ROS or sends requests.

Example: python3 test/benchmark_native_preprocessing.py --map-image map.pgm
Each backend runs in a separate process so peak RSS is not inherited from the
other backend. Times exclude Python/module startup and image loading.
"""
import argparse
import contextlib
import gc
import hashlib
import importlib.util
import io
import json
import pathlib
import resource
import statistics
import subprocess
import sys
import time
from unittest import mock

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-image')
    parser.add_argument('--rows', type=int, default=1261)
    parser.add_argument('--cols', type=int, default=1647)
    parser.add_argument('--angle', type=float, default=-7.834)
    parser.add_argument('--base-radius', type=int, default=30)
    parser.add_argument('--aligned-radius', type=int, default=14)
    parser.add_argument('--max-extent', type=int, default=60)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--worker', choices=['python', 'cpp'])
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('repeats must be positive')
    if not args.map_image and (args.rows < 3 or args.cols < 3):
        parser.error('synthetic rows and cols must be at least 3')
    if args.worker is None:
        results = {}
        for backend in ['python', 'cpp']:
            command = [sys.executable, str(pathlib.Path(__file__).resolve()),
                       *sys.argv[1:], '--worker', backend]
            results[backend] = json.loads(subprocess.check_output(command))
        for key in ['base_digest', 'stage_digest']:
            if results['python'][key] != results['cpp'][key]:
                raise RuntimeError('Backend geometry mismatch: ' + key)
        print(json.dumps(results, indent=2))
        return

    root = pathlib.Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('mst_preprocessing_benchmark', root / 'mst27.py')
    mst = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mst
    spec.loader.exec_module(mst)
    if args.worker == 'cpp' and mst._native_preprocessing() is None:
        raise RuntimeError('Build the new C++ extension before benchmarking')
    if args.map_image:
        from PIL import Image
        image = np.asarray(Image.open(args.map_image).convert('L'))
        grid = np.ascontiguousarray(image < 250, dtype=np.uint8)
    else:
        grid = np.zeros((args.rows, args.cols), np.uint8)
        grid[0,:] = grid[-1,:] = grid[:,0] = grid[:,-1] = 1
        for r in range(80, args.rows - 40, 120):
            for c in range(80, args.cols - 40, 140):
                grid[r:r+12,c:c+14] = 1
        if args.rows > 500:
            grid[400:405,40:args.cols//2] = 1
        rng = np.random.RandomState(33)
        # Fragmented returns stress the actual per-component preparation work.
        grid[rng.randint(1,args.rows-1,1000),rng.randint(1,args.cols-1,1000)] = 1

    result = {'backend': args.worker, 'shape': list(grid.shape),
              'occupied': int(np.count_nonzero(grid)),
              'angle': args.angle, 'base_radius': args.base_radius,
              'aligned_radius': args.aligned_radius, 'max_extent': args.max_extent}
    backend_context = mock.patch.object(mst, '_mst27_cpp', None) if args.worker == 'python' else contextlib.nullcontext()
    with backend_context, contextlib.redirect_stdout(io.StringIO()):
        for label, operation in [
            ('base', lambda: mst.ObstacleProcessor.inflate_obstacles(grid, args.base_radius)),
            ('stage', lambda: mst.ObstacleProcessor.build_angle_aligned_inflated(
                grid, args.angle, args.aligned_radius, args.base_radius, args.max_extent))]:
            times = []
            for _ in range(args.repeats):
                gc.collect()
                started = time.perf_counter()
                output = operation()
                times.append((time.perf_counter() - started) * 1000)
                digest = hashlib.sha256(np.asarray(output, dtype=np.uint8).tobytes()).hexdigest()
                del output
            result[label + '_ms'] = {'first': times[0], 'median': statistics.median(times), 'samples': times}
            result[label + '_digest'] = digest
    result['peak_rss_mib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    print(json.dumps(result))


if __name__ == '__main__':
    main()
