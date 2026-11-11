import unittest
from types import SimpleNamespace as NS
import numpy as np
from ee3033_search_policy.stamped_localization import (
    LocalizationMemory, map_ground_point, rotation_from_quaternion,
)


class TestStampedLocalization(unittest.TestCase):
    def test_ground_intersection(self):
        np.testing.assert_allclose(map_ground_point([0, 0, 1], [1, 0, -1], 0), [1, 0])

    def test_horizon_upward_and_range(self):
        for ray in ([1, 0, 0], [1, 0, 1], [100, 0, -1]):
            self.assertIsNone(map_ground_point([0, 0, 1], ray, 0))

    def test_quaternion_validation(self):
        np.testing.assert_allclose(rotation_from_quaternion(0, 0, 0, 2), np.eye(3))
        with self.assertRaises(ValueError):
            rotation_from_quaternion(0, 0, 0, 0)

    def test_episode_stamp_filter(self):
        m = LocalizationMemory()
        m.configure('new', 39, 'bottle')
        o = NS(episode_id='old', class_id=39, class_name='bottle',
               header=NS(stamp=NS(sec=10, nanosec=0)))
        self.assertFalse(m.accept(o, 10))
        o.episode_id = 'new'
        self.assertTrue(m.accept(o, 10))
        self.assertFalse(m.accept(o, 10))
        m.configure('next', 39, 'bottle')
        self.assertEqual(len(m.samples), 0)
        self.assertFalse(m.accept(o, 10))

    def test_bearing_only_triangulation(self):
        m = LocalizationMemory()
        for i, o in enumerate(([0, 0, 1], [0, 1, 1], [0, -.5, 1])):
            direction = np.array([2, 0, .5])-o
            m.add(10+i*.1, o, direction, None, .9)
        p, cov, method = m.estimate()
        np.testing.assert_allclose(p, [2, 0], atol=1e-6)
        self.assertIn('multiview', method)
        self.assertTrue(np.all(np.linalg.eigvalsh(cov) > 0))

    def test_identical_origins_not_multiview(self):
        m = LocalizationMemory()
        for i in range(4):
            m.add(10+i*.1, [0, 0, 1], [1, 0, 0], None, .9)
        self.assertIsNone(m.estimate()[0])

    def test_track_change_clears_history(self):
        m = LocalizationMemory()
        m.add(10, [0, 0, 1], [1, 0, 0], [1, 0], .9, 'a')
        m.add(10.1, [0, 1, 1], [1, 0, 0], [2, 1], .9, 'b')
        self.assertEqual(len(m.samples), 1)


if __name__ == '__main__':
    unittest.main()
