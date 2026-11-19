import math
import unittest
from safety import gate_reason, missing_calibration, pixel_to_world, velocity_to_pwm, world_to_pixel


class NavigationSafetyTests(unittest.TestCase):
    def settings(self):
        return dict(width_m=0.4, length_m=0.5, wheel_base_m=0.3,
                    measured_speed_at_full_pwm_mps=1.0, directions_verified=True, base_frame_verified=True,
                    invert_left=False, invert_right=True, autonomous_max_pwm=25.0)

    def test_missing_measurements_block_hardware(self):
        self.assertIn('wheel_base_m', missing_calibration({}))
        s = self.settings(); s['width_m'] = float('nan')
        self.assertIn('width_m', missing_calibration(s))

    def test_direction_confirmation_required(self):
        s = self.settings(); s['directions_verified'] = False
        self.assertIn('directions_verified', missing_calibration(s))

    def test_base_frame_confirmation_required(self):
        s = self.settings(); s['base_frame_verified'] = False
        self.assertTrue(any(v.startswith('base_frame_verified') for v in missing_calibration(s)))

    def test_pwm_keeps_right_inversion(self):
        self.assertEqual(velocity_to_pwm(0.1, 0, self.settings()), (10., -10.))

    def test_pwm_cap_preserves_ratio(self):
        left, right = velocity_to_pwm(1, 1, self.settings())
        self.assertLessEqual(max(abs(left), abs(right)), 25.0)
        self.assertAlmostEqual(abs(left/right), 0.85/1.15)

    def test_nonfinite_command_rejected(self):
        with self.assertRaises(ValueError):
            velocity_to_pwm(float('nan'), 0, self.settings())

    def test_each_stale_input_blocks(self):
        self.assertTrue(gate_reason(False, 0, 0, 0, 2))
        for ages in ((0.36, 0, 0), (0, 0.41, 0), (0, 0, 0.51)):
            self.assertTrue(gate_reason(True, *ages, 2))
        self.assertEqual(gate_reason(True, 0.1, 0.1, 0.1, 2), '')

    def test_missing_or_near_obstacle_blocks(self):
        for nearest in (None, 0.49, float('nan')):
            self.assertTrue(gate_reason(True, 0, 0, 0, nearest))

    def test_coordinate_roundtrip_with_rotation(self):
        origin = [-8.55, -16.4, math.pi/4]
        x, y = pixel_to_world(123, 345, 501, 635, 0.05, origin)
        col, row = world_to_pixel(x, y, 635, 0.05, origin)
        self.assertAlmostEqual(col, 123)
        self.assertAlmostEqual(row, 345)
        with self.assertRaises(ValueError):
            pixel_to_world(-1, 0, 501, 635, 0.05, origin)


if __name__ == '__main__':
    unittest.main()
