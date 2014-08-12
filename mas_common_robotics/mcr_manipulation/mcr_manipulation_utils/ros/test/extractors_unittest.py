#!/usr/bin/env python
"""
Test unit for the functions in the extractors.py module.

"""

PKG = 'mcr_manipulation_utils_ros'

import unittest
import rosunit
import sensor_msgs.msg
import mcr_manipulation_utils_ros.extractors


class TestExtractors(unittest.TestCase):
    """
    Tests functions used in the extractors.py module.

    """
    def test_extract_positions(self):
        """
        Tests that the 'extract_positions' function returns
        only the joint position values of the specified joints.

        """
        joint_state = sensor_msgs.msg.JointState()
        joint_state.name = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        joint_state.position = [
            -1.4625, -1.4834, -2.5729, -2.0431, 0.4070, 1.8124, -2.8385
        ]

        joint_names = ['arm_6_joint', 'arm_4_joint', 'arm_2_joint']

        expected_result = [1.8124, -2.0431, -1.4834]

        result = mcr_manipulation_utils_ros.extractors.extract_positions(
            joint_state, joint_names
        )

        for i, res in enumerate(result):
            self.assertAlmostEqual(res, expected_result[i])

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_extractors', TestExtractors)
