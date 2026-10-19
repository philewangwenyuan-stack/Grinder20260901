#!/usr/bin/env python3

import os
import sys
import unittest
import json
import tempfile


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from grinder_scheduler.map_service import MapService
from grinder_scheduler.models import (
    is_planning_direction,
    normalize_planning_direction,
    planning_direction_axis,
    explicit_region_start,
)


class PlanningDirectionTest(unittest.TestCase):
    def test_unspecified_region_start_stays_empty_and_explicit_corner_survives_reload(self):
        service = MapService()
        regions = service._work_regions
        points = [dict(x=0., y=0.), dict(x=8., y=0.),
                  dict(x=8., y=6.), dict(x=0., y=6.)]
        region, _ = service._upsert_region(regions, dict(region_id='A', points=points),
                                          'A', 'work', 1)
        self.assertEqual({}, region['start_pose'])
        corner = dict(x=0., y=6.)
        service._upsert_region(regions, dict(region_id='A', start_pose=corner), 'A', 'work', 1)
        with tempfile.TemporaryDirectory() as directory:
            service.save_local_state(directory)
            restored = MapService()
            restored.load_local_state(directory)
        actual = restored.get_overlay_regions()['work_regions'][0]
        self.assertTrue(actual['start_pose_explicit'])
        self.assertEqual(0., explicit_region_start(actual)['x'])
        self.assertEqual(6., explicit_region_start(actual)['y'])

    def test_legacy_default_start_migrates_for_axis_aligned_and_rotated_regions(self):
        for points in (
            [dict(x=0., y=0.), dict(x=8., y=0.), dict(x=8., y=6.), dict(x=0., y=6.)],
            [dict(x=0., y=0.), dict(x=8., y=1.), dict(x=7., y=6.), dict(x=-1., y=5.)],
        ):
            start = dict(x=min(p['x'] for p in points), y=max(p['y'] for p in points))
            region = dict(region_id='A', points=points, start_pose=start)
            self.assertEqual({}, explicit_region_start(region))
            with tempfile.TemporaryDirectory() as directory:
                with open(os.path.join(directory, 'map_overlay_state.json'), 'w') as handle:
                    json.dump(dict(regions=dict(work_regions=[region])), handle)
                service = MapService()
                service.load_local_state(directory)
            self.assertEqual({}, service.get_overlay_regions()['work_regions'][0]['start_pose'])

    def test_legacy_user_start_away_from_default_corner_is_preserved(self):
        region = dict(points=[dict(x=0.,y=0.),dict(x=8.,y=0.),dict(x=8.,y=6.)],
                      start_pose=dict(x=4.,y=2.))
        self.assertEqual(region['start_pose'], explicit_region_start(region))

    def test_all_supported_directions_are_preserved(self):
        for direction in ("x", "-x", "y", "-y"):
            self.assertTrue(is_planning_direction(direction))
            self.assertEqual(direction, normalize_planning_direction(direction))

    def test_axis_ignores_travel_sign(self):
        self.assertEqual("x", planning_direction_axis("-x"))
        self.assertEqual("y", planning_direction_axis("-y"))

    def test_invalid_direction_uses_supported_fallback(self):
        self.assertEqual("-y", normalize_planning_direction("invalid", "-y"))
        self.assertEqual("x", normalize_planning_direction("invalid", "invalid"))

    def test_map_region_preserves_negative_direction(self):
        service = MapService()
        regions = {}
        region, _ = service._upsert_region(
            regions,
            {
                "region_id": "work_1",
                "name": "work_1",
                "global_direction": "-y",
                "points": [
                    {"x": 0.0, "y": 0.0},
                    {"x": 1.0, "y": 0.0},
                    {"x": 1.0, "y": 1.0},
                ],
            },
            "work",
            "work_region",
            1,
        )
        self.assertEqual("-y", region["global_direction"])


if __name__ == "__main__":
    unittest.main()
