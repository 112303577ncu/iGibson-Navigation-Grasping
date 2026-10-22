"""Offline counterexamples and geometry checks; no robot or network fixtures."""
import math
import tempfile
import unittest
from pathlib import Path

from integration import g2_offline_workspace_plan as planner


class WorkspacePlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.workspace = planner.Workspace()
        cls.camera = planner.Camera()
        cls.report = planner.build(cls.workspace, cls.camera)

    def test_frame_round_trip_in_rotated_room(self):
        for point in ((0., 0.), (.95, .95), self.workspace.size):
            recovered = self.workspace.to_room(self.workspace.to_map(point))
            self.assertLess(math.dist(recovered, point), 1e-12)

    def test_centre_legal_is_insufficient_for_turning(self):
        candidates = [(i / 100, .8) for i in range(35, 60)]
        self.assertTrue(any(self.workspace.clear(p) and not self.workspace.clear(p, True)
                            for p in candidates))

    def test_unknown_map_under_outline_blocks_centre(self):
        point = (.9, .9)
        self.assertTrue(self.workspace.clear(point))
        x, y = self.workspace.to_map(point)
        policy = self.workspace.policy
        col = math.floor((x - policy.origin[0]) / policy.resolution)
        row = policy.height - 1 - math.floor((y - policy.origin[1]) / policy.resolution)
        original = self.workspace.map_clearance[row, col]
        try:
            self.workspace.map_clearance[row, col] = .1
            self.workspace.clear.cache_clear()
            self.assertFalse(self.workspace.clear(point))
        finally:
            self.workspace.map_clearance[row, col] = original
            self.workspace.clear.cache_clear()

    def test_bin_envelope_and_invalid_positions_are_forbidden(self):
        for p in ((1.65, 1.70), (-.1, 1.), (float('nan'), 1.), (1., float('inf'))):
            self.assertFalse(self.workspace.clear(p))

    def test_each_sampled_target_avoids_entire_patrol_sweep(self):
        route = self.workspace.route
        for case in self.report['pickup_cases']:
            distance = min(planner.segment_distance(case['target_room_m'], a, b)
                           for a, b in zip(route, route[1:] + route[:1]))
            self.assertGreaterEqual(distance + 1e-10,
                                    planner.RADIUS + planner.BUFFER + planner.SUGARBOX_RADIUS)

    def test_pickup_and_reverse_are_admitted_before_leaving_route(self):
        for case in self.report['pickup_cases']:
            self.assertTrue(self.workspace.corridor(case['station_room_m'], case['stop_room_m']))
            self.assertTrue(self.workspace.corridor(case['stop_room_m'], case['station_room_m']))
            self.assertLessEqual(case['reverse_to_station_m'], .60)
            self.assertAlmostEqual(math.dist(case['target_room_m'], case['stop_room_m']), .34)
            self.assertFalse(case['turn_after_pickup'])

    def test_too_close_far_or_off_camera_target_is_rejected(self):
        station = self.workspace.route[0]
        for target in (station, (.5, .6), (1.7, 1.7), (0., 0.)):
            self.assertIsNone(planner.pickup(self.workspace, self.camera, target, station))

    def test_generated_maps_expand_translation_without_enabling_hardware(self):
        report = dict(self.report)
        with tempfile.TemporaryDirectory() as folder:
            planner.write_masks(self.workspace, report, Path(folder))
            self.assertTrue((Path(folder) / 'navigation_center_mask.proposal.pgm').is_file())
        comparison = report['mask_comparison']
        self.assertGreater(comparison['new_centre_area_m2'], comparison['old_centre_area_m2'])
        self.assertLess(comparison['turn_area_m2'], comparison['new_centre_area_m2'])
        self.assertFalse(comparison['active_files_modified'])
        self.assertFalse(report['hardware_validated'])
        self.assertFalse(report['runtime_enforced'])
        self.assertFalse(report['error_bounds_verified'])


if __name__ == '__main__':
    unittest.main()
