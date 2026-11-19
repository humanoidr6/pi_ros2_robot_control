import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from extract_geometry import origin_matrix, stl_vertices


class GeometryTests(unittest.TestCase):
    def test_urdf_transform_rotates_before_translation(self):
        origin = ET.fromstring('<origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>')
        actual = origin_matrix(origin) @ np.array([1, 0, 0, 1])
        np.testing.assert_allclose(actual, [1, 3, 3, 1], atol=1e-12)

    def test_binary_stl_vertices_and_truncation(self):
        triangle = struct.pack('<12fH', 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'triangle.stl'
            raw = b' '*80+struct.pack('<I', 1)+triangle
            path.write_bytes(raw)
            np.testing.assert_array_equal(stl_vertices(path), [[0, 0, 0], [1, 0, 0], [0, 1, 0]])
            path.write_bytes(raw[:-1])
            with self.assertRaises(ValueError):
                stl_vertices(path)

    def test_saved_footprint_contains_every_zero_pose_link(self):
        report = json.loads(Path(__file__).with_name('body_geometry.json').read_text())
        footprint = np.array(report['footprint_about_axle_midpoint_m'])
        low, high = footprint.min(0), footprint.max(0)
        axle = np.array(report['axle_midpoint_in_cad_base_link_m'])[:2]
        for bounds in report['link_bounds'].values():
            self.assertTrue(np.all(np.array(bounds['min_m'])[:2]-axle >= low-1e-8))
            self.assertTrue(np.all(np.array(bounds['max_m'])[:2]-axle <= high+1e-8))
        self.assertGreater(report['circumscribed_radius_about_axle_m'], math.hypot(report['length_m'], report['width_m'])/2)

    def test_tread_centres_not_joint_reference_origins(self):
        report = json.loads(Path(__file__).with_name('body_geometry.json').read_text())
        self.assertAlmostEqual(report['tyre_centre_separation_m'], 0.3846, places=6)
        self.assertAlmostEqual(report['wheel_joint_origin_separation_m'], 0.3786, places=6)


if __name__ == '__main__':
    unittest.main()
