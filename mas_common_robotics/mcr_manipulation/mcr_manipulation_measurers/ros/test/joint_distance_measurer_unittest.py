#!/usr/bin/env python
"""
Test unit for the functions/methods used in joint_distance_measurer.py module.

"""

import unittest
import rosunit
import mcr_manipulation_measurers_ros.joint_distance_measurer \
    as joint_distance_measurer

PKG = 'mcr_manipulation_measurers'


class TestJointDistanceMeasurer(unittest.TestCase):
    """
    Tests methods used in the joint_distance_measurer.py module.

    """
    def test_measure_joint_distances(self):
        """
        Tests that the 'measure_joint_distances' function returns
        the correct distance between joints.

        """
        actual_1 = [-1.4625, -1.4834, -2.5729]
        actual_2 = [-1.4625, -1.4834, -2.5729]
        actual_3 = [-1.4625, -1.4834, -2.5729, -2.0431,
                    0.4070, 1.8124, -2.8385]
        reference_1 = [-2.5729, 0.0431, 0.258]
        reference_2 = [-2.5729, -0.0431, 2.478]
        reference_3 = [-1.4625, -0.4834, -2.5729, -2.0431,
                       0.4070, -1.8124, -0.8385]

        expected_result_1 = [-1.1104, 1.5265, 2.8309]
        expected_result_2 = [-1.1104, 1.4403, 5.0509]
        expected_result_3 = [0.0, 1.0, 0.0, 0.0, 0.0, -3.6248, 2.0]

        result_1 = joint_distance_measurer.measure_joint_distances(
            actual_1, reference_1
        )
        result_2 = joint_distance_measurer.measure_joint_distances(
            actual_2, reference_2
        )
        result_3 = joint_distance_measurer.measure_joint_distances(
            actual_3, reference_3
        )

        for i, result in enumerate(result_1):
            self.assertAlmostEqual(result, expected_result_1[i])
        for i, result in enumerate(result_2):
            self.assertAlmostEqual(result, expected_result_2[i])
        for i, result in enumerate(result_3):
            self.assertAlmostEqual(result, expected_result_3[i])


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_joint_distance_measurer', TestJointDistanceMeasurer)
