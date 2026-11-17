import unittest
from bridge import motion_allowed


class SafetyTests(unittest.TestCase):
    def test_stale_lidar_blocks_all_directions(self):
        for left, right in [(20, 20), (-20, -20), (20, -20), (-20, 20)]:
            self.assertFalse(motion_allowed(left, right, 0.41, 2)[0])

    def test_obstacle_or_missing_ranges_blocks(self):
        for nearest in (None, 0.2, 0.49):
            self.assertFalse(motion_allowed(20, 20, 0.1, nearest)[0])

    def test_clear_fresh_scan_allows_motion(self):
        self.assertTrue(motion_allowed(20, 20, 0.1, 1.0)[0])

    def test_stop_always_allowed(self):
        self.assertTrue(motion_allowed(0, 0, 100, None)[0])


if __name__ == '__main__':
    unittest.main()
