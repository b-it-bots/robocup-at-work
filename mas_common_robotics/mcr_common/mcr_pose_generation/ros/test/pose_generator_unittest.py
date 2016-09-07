#!/usr/bin/env python
"""
Test unit for the functions in the pose_generator_utils.py module.

To run it, type:
py.test [file_name]

"""

PKG = 'mcr_pose_generation'

import numpy
import numpy.testing as testing
import unittest
import rosunit
import mcr_pose_generation_ros.pose_generator_utils as pose_generator_utils


class TestPoseGenerator(unittest.TestCase):
    """
    Tests methods used in the pose_generator.py module.

    """
    def test_generate_samples_basic(self):
        """
        Tests the function 'generate_samples' with a basic case.

        """
        desired_result = numpy.linspace(0.0, 1.0, num=11)

        min_value = 0.0
        max_value = 1.0
        step = 0.1

        actual_result = pose_generator_utils.generate_samples(min_value, max_value, step)
        testing.assert_almost_equal(actual_result, desired_result)

    def test_generate_samples_no_interval(self):
        """
        Tests that the function 'generate_samples' returns a correct
        result when the minimum and maximum values are the same.

        """
        desired_result = 0.45

        min_value = 0.45
        max_value = 0.45
        step = 0.1

        actual_result = pose_generator_utils.generate_samples(min_value, max_value, step)
        testing.assert_almost_equal(actual_result, desired_result)

    def test_generate_samples_max_value_exceeded(self):
        """
        Tests that the function 'generate_samples' returns a correct
        result when the step size would generate a sample greater than
        the maximum value.

        """
        desired_result = [0.3, 0.39, 0.48]

        min_value = 0.3
        max_value = 0.48
        step = 0.1

        actual_result = pose_generator_utils.generate_samples(min_value, max_value, step)
        testing.assert_almost_equal(actual_result, desired_result)

    def test_generate_samples_max_samples_exceeded(self):
        """
        Tests that the function 'generate_samples' returns a correct
        result when the step size would generate more samples than allowed.

        """
        max_samples = 20
        min_value = 0.3
        max_value = 0.6
        step = 0.01

        # only test the length is the same, not the contents of the array
        expected_result = [0.0] * max_samples

        result = pose_generator_utils.generate_samples(
            min_value, max_value, step, max_samples=max_samples)

        self.assertEqual(len(result), len(expected_result))


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'pose_generator_unittest', TestPoseGenerator)
