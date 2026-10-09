"""Before/after full-adapter benchmark, isolated worker processes, no ROS.

Use --native-dir to load a freshly compiled Linux extension into both versions.
Baseline files were captured before this change; input and path digest must match.
"""
import argparse
import contextlib
import hashlib
import importlib.util
import io
import json
import os
import pathlib
import statistics
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent


def load(name,path):
    spec = importlib.util.spec_from_file_location(name,str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def worker(args):
    if args.native_dir:
        sys.path.insert(0,args.native_dir)
        import _mst27_cpp
    helper = load('adapter_benchmark_helpers',ROOT/'catkin_ws/src/grinder_scheduler/test/test_planner_preprocessing.py')
    np = helper.np
    if args.worker == 'baseline':
        planner = load('mst27_before_optimization',HERE/'baseline/mst27.py')
        module = helper.load_adapter(HERE/'baseline/planner_adapter.py')
    else:
        planner,module = helper.mst,helper.adapter_module
    if args.backend == 'python':
        planner._mst27_cpp = None
    grid = np.zeros((args.size,args.size),dtype=np.int16)
    grid[0,:] = grid[-1,:] = grid[:,0] = grid[:,-1] = 100
    regions = []
    pitch,span = 170,140
    columns = max(1,(args.size-30)//pitch)
    for index in range(args.regions):
        row,col = 20+(index//columns)*pitch,20+(index%columns)*pitch
        if row+span >= args.size or col+span >= args.size:
            raise ValueError('map too small for benchmark regions')
        grid[row+62:row+67,col+62:col+67] = 100
        x0,y0,x1,y1 = col*.05,row*.05,(col+span)*.05,(row+span)*.05
        regions.append(dict(region_id=str(index),enabled=True,
            points=[dict(x=x0,y=y0),dict(x=x1,y=y0),dict(x=x1,y=y1),dict(x=x0,y=y1)],
            start_pose=dict(x=x0+.8,y=y0+.8),end_pose=dict(x=x1-.8,y=y1-.8),
            global_direction='x'))
    cfg = helper.TaskConfigModel(work_regions=regions,
        selected_work_region_ids=[r['region_id'] for r in regions],
        vehicle_width=.8,vehicle_length=1.4,default_path_spacing=.7,
        inflation_radius=.6,aligned_obstacle_inflation=.7,aligned_obstacle_max_extent=3.,
        endpoint_margin=.2,overlap_ratio=0.,turn_radius=.1)
    info = dict(width=args.size,height=args.size,resolution=.05,origin_x=0.,origin_y=0.,frame_id='map')
    contexts = []
    original_prepare = getattr(planner.MultiStagePathPlanner,'prepare_map',None)
    if original_prepare:
        def prepare(self,*values,**kwargs):
            result = original_prepare(self,*values,**kwargs)
            contexts.append(result)
            return result
        planner.MultiStagePathPlanner.prepare_map = prepare
    adapter = helper.adapter(module,planner)
    started = time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):
        result = adapter.plan('benchmark',grid,info,cfg)
    elapsed_ms = (time.perf_counter()-started)*1000
    digest = hashlib.sha256(json.dumps(result.points,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    peak_rss = None
    try:
        import resource
        peak_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    except ImportError:
        try:
            import psutil
            peak_rss = psutil.Process().memory_info().peak_wset
        except (ImportError,AttributeError):
            pass
    backend = 'cpp' if adapter._last_runtime_backend == 'cpp' else 'python'
    print(json.dumps(dict(mode=args.worker,backend=backend,elapsed_ms=elapsed_ms,
        peak_rss_bytes=peak_rss,path_points=len(result.points),path_digest=digest,
        cache=contexts[-1].stats() if contexts else None)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--size',type=int,default=1000)
    parser.add_argument('--regions',type=int,default=10)
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--backend',choices=('cpp','python'),default='cpp')
    parser.add_argument('--native-dir',default='')
    parser.add_argument('--worker',choices=('baseline','optimized'))
    parser.add_argument('--output',default=str(HERE/'benchmark.json'))
    args = parser.parse_args()
    if args.worker:
        worker(args)
        return
    records = []
    for _ in range(args.repeats):
        for mode in ('baseline','optimized'):
            command = [sys.executable,str(pathlib.Path(__file__).resolve()),'--worker',mode,
                '--size',str(args.size),'--regions',str(args.regions),'--backend',args.backend,
                '--native-dir',args.native_dir]
            completed = subprocess.run(command,check=True,stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,encoding='utf-8')
            record = json.loads(completed.stdout)
            if args.backend == 'cpp' and record['backend'] != 'cpp':
                raise RuntimeError('C++ requested but unavailable')
            records.append(record)
            print('{} {:.1f}ms points={}'.format(mode,record['elapsed_ms'],record['path_points']),flush=True)
    if not all(record['path_points'] for record in records):
        raise RuntimeError('benchmark produced empty paths')
    if len(set(record['path_digest'] for record in records)) != 1:
        raise RuntimeError('baseline and optimized path records differ')
    medians = {mode:statistics.median([r['elapsed_ms'] for r in records if r['mode']==mode])
        for mode in ('baseline','optimized')}
    report = dict(size=args.size,regions=args.regions,repeats=args.repeats,
        platform=sys.platform,python=sys.version,medians_ms=medians,
        time_reduction_pct=(1-medians['optimized']/medians['baseline'])*100,
        path_records_equal=True,records=records)
    pathlib.Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({key:report[key] for key in ('medians_ms','time_reduction_pct','path_records_equal')}),flush=True)


if __name__ == '__main__':
    for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS'):
        os.environ.setdefault(name,'1')
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    main()
