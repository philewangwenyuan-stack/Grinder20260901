#!/usr/bin/env python3
"""JSON scan + 4x4 scan_to_base transform -> production scan_replay executable.

JSON: angle_min, angle_increment, range_min, range_max, ranges (null=invalid),
scan_to_base (4x4 row-major), footprint_bounds [minx,maxx,miny,maxy],
optional self_polygon [[x,y],...], front_clearance, side_clearance.
The transform must be measured at the scan timestamp, not taken from another run.
"""
import argparse
import json
import math
import subprocess


def replay_input(data):
    transform = data['scan_to_base']
    if len(transform) != 4 or any(len(row) != 4 for row in transform):
        raise ValueError('scan_to_base must be 4x4')
    if not all(math.isfinite(value) for row in transform for value in row):
        raise ValueError('scan_to_base must be finite')
    polygon = data.get('self_polygon', [])
    header = list(data['footprint_bounds']) + [data.get('front_clearance',.15),
        data.get('side_clearance',.1), len(polygon)]
    lines = [' '.join(map(str,header))] + ['{} {}'.format(*point) for point in polygon]
    for i, distance in enumerate(data['ranges']):
        if distance is None or not math.isfinite(distance) or not data['range_min'] <= distance <= data['range_max']:
            continue
        angle = data['angle_min']+i*data['angle_increment']
        point = (distance*math.cos(angle), distance*math.sin(angle), 0, 1)
        x,y = (sum(row[j]*point[j] for j in range(4)) for row in transform[:2])
        lines.append('{} {}'.format(x,y))
    return '\n'.join(lines)+'\n'


def scan_from_bag(path, frame, bounds, scan_index=0):
    # Offline only: debug=False avoids advertising a TF service or contacting a master.
    import rosbag
    import rospy
    import tf2_ros
    from tf.transformations import quaternion_matrix
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(600), debug=False)
    with rosbag.Bag(path) as bag:
        for topic, message, _ in bag.read_messages(topics=['/tf', '/tf_static']):
            for transform in message.transforms:
                if topic == '/tf_static': buffer.set_transform_static(transform, 'recording')
                else: buffer.set_transform(transform, 'recording')
        selected = None
        for i, (_, scan, _) in enumerate(bag.read_messages(topics=['/scan'])):
            if i == scan_index: selected = scan; break
        if selected is None: raise ValueError('scan index not found')
        # lookup_transform_core is immediate; never waits for live TF.
        transform = buffer.lookup_transform_core(frame, selected.header.frame_id, selected.header.stamp)
        rotation = transform.transform.rotation
        matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        translation = transform.transform.translation
        matrix[:3, 3] = [translation.x, translation.y, translation.z]
        return dict(angle_min=selected.angle_min, angle_increment=selected.angle_increment,
            range_min=selected.range_min, range_max=selected.range_max,
            ranges=[float(x) if math.isfinite(x) else None for x in selected.ranges],
            scan_to_base=matrix.tolist(), footprint_bounds=bounds)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scan_json', help='JSON input, or bag when --bag is supplied')
    parser.add_argument('replay_executable')
    parser.add_argument('--bag', action='store_true')
    parser.add_argument('--scan-index', type=int, default=0)
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--bounds', nargs=4, type=float, metavar=('MIN_X','MAX_X','MIN_Y','MAX_Y'))
    parser.add_argument('--self-polygon-json', help='Measured XY polygon; disabled by default')
    args = parser.parse_args()
    if args.bag:
        if args.bounds is None: parser.error('--bag requires measured --bounds')
        data = scan_from_bag(args.scan_json,args.base_frame,args.bounds,args.scan_index)
    else:
        with open(args.scan_json) as stream: data = json.load(stream)
    if args.self_polygon_json:
        with open(args.self_polygon_json) as stream: data['self_polygon'] = json.load(stream)
    subprocess.run([args.replay_executable], input=replay_input(data), text=True, check=True)
