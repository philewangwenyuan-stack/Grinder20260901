#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from grinder_scheduler.map_service import MapService
from grinder_scheduler.models import (
    is_planning_direction,
    normalize_planning_direction,
    planning_direction_axis,
)


class PlanningDirectionTest(unittest.TestCase):
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
