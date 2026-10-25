#!/usr/bin/env python3
"""Offline paired arc-window benchmark; no ROS imports, no board connection.

Run from any directory: python benchmark_sparse_segments.py --baseline-ref <commit>
Baseline/current use identical dense input to isolate indexing improvements.
Local sampling timings are separate and exclude ROS objects/serialization.
"""
import argparse
import json
import platform
import statistics
import subprocess
import time
from pathlib import Path

from test_sparse_segments import SOURCE, node, points, scheduler_class
from grinder_scheduler.path_geometry import sample_segment


def measure(call, iterations):
    for _ in range(5):
        call()
    samples = []
    for _ in range(iterations):
        start = time.perf_counter()
        call()
        samples.append((time.perf_counter() - start) * 1000.)
    return round(statistics.median(samples), 5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-ref', required=True)
    parser.add_argument('--iterations', type=int, default=101)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error('--iterations must be positive')
    root = SOURCE.parents[5]
    relative = SOURCE.relative_to(root).as_posix()
    baseline_sha = subprocess.check_output(['git', 'rev-parse', args.baseline_ref], cwd=str(root), text=True).strip()
    old_source = subprocess.check_output(['git', 'show', baseline_sha + ':' + relative], cwd=str(root), text=True, encoding='utf-8')
    classes = {'before': scheduler_class(old_source), 'after': scheduler_class()}
    rows = []
    for count in (711, 20000, 50000):
        coords = [(i * .3, 0.) for i in range(count)]
        start_s, end_s = count * .3 * .5, count * .3 * .5 + 30.
        row = {'global_points': count, 'window_m': 30.}
        for version, cls in classes.items():
            n = node(points(coords), cls)
            n._ensure_path_arc_lengths()
            row[version] = {
                'slice_ms': measure(lambda:n._slice_path_points_by_arc(start_s, end_s), args.iterations),
                'projection_ms': measure(lambda:n._project_pose_to_path_progress(start_s+1., .1, start_s, end_s), args.iterations),
            }
            if version == 'after':
                window = n._slice_path_points_by_arc(start_s, end_s)
                row['local_sample_ms'] = measure(lambda:sample_segment(window,.3), args.iterations)
                row['local_output_points'] = len(sample_segment(window,.3))
        rows.append(row)
    n = node(points([(0,0),(30,0),(30,10)]))
    n._ensure_path_arc_lengths()
    corner_window=n._slice_path_points_by_arc(25,35)
    sparse_example = {'global_points':3, 'window_m':10.,
        'slice_ms':measure(lambda:n._slice_path_points_by_arc(25,35),args.iterations),
        'projection_ms':measure(lambda:n._project_pose_to_path_progress(29,.1,25,35),args.iterations),
        'local_sample_ms':measure(lambda:sample_segment(corner_window,.3),args.iterations),
        'local_output_points':len(sample_segment(corner_window,.3))}
    result={'platform':platform.platform(),'python':platform.python_version(),
        'baseline_commit':baseline_sha,'iterations':args.iterations,'statistic':'median',
        'scope':'workstation production AST methods; cached arc lengths; synthetic polyline; excludes ROS publish, board and APP',
        'same_input_rows':rows,'sparse_corner_example':sparse_example}
    encoded=json.dumps(result,ensure_ascii=False,indent=2)
    if args.output:
        args.output.write_text(encoded+'\n',encoding='utf-8')
    print(encoded)


if __name__ == '__main__':
    main()
