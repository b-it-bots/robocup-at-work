#!/usr/bin/env python
"""
Test unit for the functions used in the trajectory_generator.py module.

"""

PKG = 'mcr_trajectory_generation'

import unittest
import rosunit
import mcr_trajectory_generation_ros.trajectory_generator_utils as utils


class TestTrajectoryGenerator(unittest.TestCase):
    """
    Tests the functions used in the trajectory_generator.py module.

    """
    def test_calculate_maximum_duration_bad_input(self):
        """
        Tests that the 'calculate_maximum_duration' function raises an exception caused
        by specifying a wrong argument.

        """
        good_point_1 = [1, 2, 3]
        good_point_2 = [1, 2, 3]
        bad_point_1 = 5.5               # it should be a list
        bad_point_2 = [5, 2]            # list is too short
        bad_scaling_factor = "1.0"      # string instead of float

        with self.assertRaises(AssertionError):
            utils.calculate_maximum_duration(good_point_1, bad_point_1)
        with self.assertRaises(AssertionError):
            utils.calculate_maximum_duration(bad_point_1, good_point_2)
        with self.assertRaises(AssertionError):
            utils.calculate_maximum_duration(good_point_1, bad_point_2)
        with self.assertRaises(AssertionError):
            utils.calculate_maximum_duration(
                good_point_1, good_point_2, bad_scaling_factor
            )

    def test_calculate_maximum_duration(self):
        """
        Tests that the 'calculate_maximum_duration' function returns the correct result.

        """
        point_1 = [1, 2, 3]
        point_2 = [1, 2, 3]
        expected = 0

        actual = utils.calculate_maximum_duration(point_1, point_2)
        self.assertEqual(actual, expected)

        point_1 = [1, 2, 3]
        point_2 = [-1, 2, 3]
        expected = 2

        actual = utils.calculate_maximum_duration(point_1, point_2)
        self.assertEqual(actual, expected)

        point_1 = [-1, 2, 8]
        point_2 = [1, 2, 3]
        expected = 5

        actual = utils.calculate_maximum_duration(point_1, point_2)
        self.assertEqual(actual, expected)

        point_1 = [-1, 2, 8]
        point_2 = [1, 2, 3]
        scaling_factor = 3.0
        expected = 15

        actual = utils.calculate_maximum_duration(point_1, point_2, scaling_factor)
        self.assertEqual(actual, expected)

    def test_calculate_discrete_difference_bad_input(self):
        """
        Tests that the 'calculate_discrete_difference' function raises an exception
        caused by specifying a wrong argument.

        """
        good_point_1 = [1, 2, 3]
        good_point_2 = [1, 2, 3]
        bad_point_1 = 5.5               # it should be a list
        bad_point_2 = [5, 2]            # list is too short
        bad_maximum_limit = "1.0"       # string instead of float

        with self.assertRaises(AssertionError):
            utils.calculate_discrete_difference(good_point_1, bad_point_1)
        with self.assertRaises(AssertionError):
            utils.calculate_discrete_difference(bad_point_1, good_point_2)
        with self.assertRaises(AssertionError):
            utils.calculate_discrete_difference(good_point_1, bad_point_2)
        with self.assertRaises(AssertionError):
            utils.calculate_discrete_difference(
                good_point_1, good_point_2, bad_maximum_limit
            )

    def test_calculate_discrete_difference(self):
        """
        Tests that the 'calculate_discrete_difference' function returns the correct result.

        """
        point_1 = [1, 2, 3]
        point_2 = [1, 2, 3]
        expected = [0, 0, 0]

        actual = utils.calculate_discrete_difference(point_1, point_2)
        self.assertEqual(actual, expected)

        point_1 = [1, 2, 3]
        point_2 = [-1, 5, 4]
        expected = [-2, 3, 1]

        actual = utils.calculate_discrete_difference(point_1, point_2)
        self.assertEqual(actual, expected)

        point_1 = [1, 2, 3]
        point_2 = [-2, 4, 15]
        expected = [-3, 2, 3]
        maximum_limit = 3.0

        actual = utils.calculate_discrete_difference(point_1, point_2, maximum_limit)
        self.assertEqual(actual, expected)

        point_1 = [1, 2, 3]
        point_2 = [-8, 4, 15]
        expected = [-3, 2, 3]
        maximum_limit = 3.0

        actual = utils.calculate_discrete_difference(point_1, point_2, maximum_limit)
        self.assertEqual(actual, expected)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_trajectory_generator', TestTrajectoryGenerator)
