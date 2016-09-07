#!/usr/bin/env python
"""
Test unit for the functions/methods used in cartesian_distance_monitor.py module.

"""

import unittest
import rosunit
import mcr_manipulation_monitors_ros.cartesian_distance_monitor \
    as cartesian_distance_monitor

PKG = 'mcr_manipulation_monitors'


class TestCartesianDistanceMonitor(unittest.TestCase):
    """
    Tests methods used in the cartesian_distance_monitor.py module.

    """
    def test_calculate_euclidean_distance(self):
        """
        Tests that the 'calculate_euclidean_distance'
        returns the correct result.

        """
        x_1 = -7.0
        y_1 = -4.0
        z_1 = 3.0

        x_2 = 17.0
        y_2 = 6.0
        z_2 = 2.5

        result = 26.004807

        self.assertAlmostEqual(
            cartesian_distance_monitor.calculate_euclidean_distance(
                x_1, y_1, z_1, x_2, y_2, z_2), result, places=6
        )

        x_1 = -0.5
        y_1 = 1.2
        z_1 = 0.06

        x_2 = 1.0
        y_2 = 2.0
        z_2 = 1.0

        result = 1.942576

        self.assertAlmostEqual(
            cartesian_distance_monitor.calculate_euclidean_distance(
                x_1, y_1, z_1, x_2, y_2, z_2), result, places=6
        )


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_cartesian_distance_monitor', TestCartesianDistanceMonitor)
