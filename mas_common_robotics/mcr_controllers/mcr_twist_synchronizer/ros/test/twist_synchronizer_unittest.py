#!/usr/bin/env python
"""
Test unit for the functions in the twist_synchronizer.py module.

"""

import numpy.testing as testing
import unittest
import rosunit
import mcr_twist_synchronizer_ros.twist_synchronizer_utils as twist_synchronizer_utils

PKG = 'mcr_twist_synchronizer'


class TestTwistSynchronizer(unittest.TestCase):
    """
    Tests methods used in the twist_synchronizer.py module.

    """
    def test_calculate_max_time_bad_input(self):
        """
        Tests that the 'calculate_max_time' function raises an exception caused
        by specifying a wrong argument.

        """
        velocity_3d = [1.0, 2.0, 3.0]
        velocity_6d = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

        error_3d = [1.0, 2.0, 3.0]
        error_6d = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

        # error and velocity have different dimensions
        with self.assertRaises(AssertionError):
            twist_synchronizer_utils.calculate_max_time(error_6d, velocity_3d)
        # error and velocity should be three-dimensional
        with self.assertRaises(AssertionError):
            twist_synchronizer_utils.calculate_max_time(error_6d, velocity_6d)
        # error and velocity should be six-dimensional,
        # since angular_synchronization is True
        with self.assertRaises(AssertionError):
            twist_synchronizer_utils.calculate_max_time(
                error_3d, velocity_3d, angular_synchronization=True
            )

    def test_calculate_max_time_linear(self):
        """
        Tests that the 'calculate_max_time' function returns the correct value,
        given only a linear velocity (i.e. a three-dimensional list).

        """
        error = [0.2, 0.2, 0.2]
        velocity_1 = [0.02, -0.8, 1.96]
        velocity_2 = [0.0001, -0.8, 1.96]
        velocity_3 = [0.0001, 0.0001, 1.96]

        desired_1 = 10
        desired_2 = 0.25
        desired_3 = 0.10204

        actual_1 = twist_synchronizer_utils.calculate_max_time(error, velocity_1)
        actual_2 = twist_synchronizer_utils.calculate_max_time(error, velocity_2)
        actual_3 = twist_synchronizer_utils.calculate_max_time(error, velocity_3)

        self.assertAlmostEqual(actual_1, desired_1, places=4)
        self.assertAlmostEqual(actual_2, desired_2, places=4)
        self.assertAlmostEqual(actual_3, desired_3, places=4)

    def test_calculate_max_time_angular(self):
        """
        Tests that the 'calculate_max_time' function returns the correct value,
        given linear and angular velocities (i.e. a six-dimensional list).

        """
        error_1 = [4.0, 2.0, 3.0, 1.0, 8.0, 5.0]
        velocity_1 = [1.0, -2.0, 3.0, -4.0, -5.0, 6.0]
        desired_1 = 4.0

        error_2 = [5.0, 4.0, 0.0, 0.0, 0.0, -6.0]
        velocity_2 = [20.0, -20.0, 3.0, -4.0, -5.0, 12.0]
        desired_2 = 0.5

        actual_1 = twist_synchronizer_utils.calculate_max_time(
            error_1, velocity_1, angular_synchronization=True
        )
        actual_2 = twist_synchronizer_utils.calculate_max_time(
            error_2, velocity_2, angular_synchronization=True
        )

        self.assertAlmostEqual(actual_1, desired_1)
        self.assertAlmostEqual(actual_2, desired_2)

    def test_calculate_sync_velocity_bad_input(self):
        """
        Tests that the 'calculate_sync_velocity' function raises an exception caused
        by specifying a wrong argument.

        """
        max_time = 1.0

        velocity_3d = [1.0, 2.0, 3.0]
        velocity_6d = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

        error_3d = [1.0, 2.0, 3.0]
        error_6d = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

        # error and velocity have different dimensions
        with self.assertRaises(AssertionError):
            twist_synchronizer_utils.calculate_sync_velocity(
                error_6d, velocity_3d, max_time
            )
        # error and velocity should be three-dimensional
        with self.assertRaises(AssertionError):
            twist_synchronizer_utils.calculate_sync_velocity(
                error_6d, velocity_6d, max_time
            )
        # error and velocity should be six-dimensional,
        # since angular_synchronization is True
        with self.assertRaises(AssertionError):
            twist_synchronizer_utils.calculate_sync_velocity(
                error_3d, velocity_3d, max_time, angular_synchronization=True
            )

    def test_calculate_sync_velocity_linear(self):
        """
        Tests that the 'calculate_sync_velocity' function returns the correct value,
        given only a linear velocity (i.e. a three-dimensional list).

        """
        max_time = 5
        error = [0.02, -0.8, 1.96]
        velocity = [0.2, -0.6, 0.5]

        desired = [0.004, -0.16, 0.392]
        actual = twist_synchronizer_utils.calculate_sync_velocity(
            error, velocity, max_time
        )

        testing.assert_almost_equal(actual, desired, decimal=4)

    def test_calculate_sync_velocity_angular(self):
        """
        Tests that the 'calculate_sync_velocity' function returns the correct value,
        given linear and angular velocities (i.e. a six-dimensional list).

        """
        max_time = 2.0

        error_1 = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        velocity_1 = [1.0, -2.0, 3.0, -4.0, -5.0, 6.0]
        desired_1 = [0.5, -1.0, 1.5, -2.0, -2.5, 3.0]

        error_2 = [1.0, 2.0, 0.0, 0.0, 0.0, 6.0]
        velocity_2 = [1.0, -2.0, 3.0, -4.0, -5.0, 6.0]
        desired_2 = [0.5, -1.0, 0.0, 0.0, 0.0, 3.0]

        actual_1 = twist_synchronizer_utils.calculate_sync_velocity(
            error_1, velocity_1, max_time, angular_synchronization=True
        )
        actual_2 = twist_synchronizer_utils.calculate_sync_velocity(
            error_2, velocity_2, max_time, angular_synchronization=True
        )

        testing.assert_almost_equal(actual_1, desired_1, decimal=4)
        testing.assert_almost_equal(actual_2, desired_2, decimal=4)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_twist_synchronizer', TestTwistSynchronizer)
