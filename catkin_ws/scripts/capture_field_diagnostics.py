#!/usr/bin/env python3
"""Bounded, read-only ROS bag capture. Never starts nodes or sends commands."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

TOPICS = ['/diagnostics', '/scan', '/tf', '/tf_static', '/rpp/collision_detail', '/rpp/collision_imminent',
          '/super_lio/registration_diagnostics', '/super_lio/localization_status',
          '/lio/odom', '/chassis/motion_safety', '/scheduler/motion_safety']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='New output directory (must not exist)')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--max-mib', type=int, default=64)
    args = parser.parse_args()
    if not 0 < args.seconds <= 300 or not 0 < args.max_mib <= 256:
        parser.error('seconds must be 0..300; max-mib must be 1..256')
    directory = Path(args.output)
    directory.mkdir(parents=True, exist_ok=False)
    process = subprocess.Popen(['rosbag', 'record', '--chunksize=256', '--buffsize=16', '-O', str(directory/'field.bag')] + TOPICS,
                               start_new_session=True)
    started = time.monotonic()
    try:
        # Budget covers recorder files, including .active. Allow up to one
        # recorder buffer + polling interval of overshoot; this is not a quota.
        while process.poll() is None:
            size = sum(p.stat().st_size for p in directory.iterdir() if p.is_file())
            if time.monotonic()-started >= args.seconds or size >= args.max_mib*1024*1024:
                break
            time.sleep(.1)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(5)
    return process.returncode


if __name__ == '__main__':
    raise SystemExit(main())
