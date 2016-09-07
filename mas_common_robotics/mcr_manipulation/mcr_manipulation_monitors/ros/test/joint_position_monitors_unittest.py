#!/usr/bin/env python
"""
Test unit for the functions/methods used in joint_position_monitors.py module.

"""

import unittest
import numpy.testing as testing
import rosunit
import sensor_msgs.msg
import mcr_manipulation_monitors_ros.joint_position_monitors \
    as joint_position_monitors

PKG = 'mcr_joint_position_monitors'


class TestJointMonitors(unittest.TestCase):
    """
    Tests methods used in the joint_position_monitors.py module.

    """
    def test_check_positions_false(self):
        """
        Tests that the 'check_joint_positions' function returns False,
        since not all joints have reached their desired position.

        """
        actual_1 = sensor_msgs.msg.JointState()
        actual_2 = sensor_msgs.msg.JointState()
        actual_3 = sensor_msgs.msg.JointState()

        reference_1 = sensor_msgs.msg.JointState()
        reference_2 = sensor_msgs.msg.JointState()
        reference_3 = sensor_msgs.msg.JointState()

        actual_1.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
        actual_2.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
        actual_3.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                         'arm_4_joint', 'arm_5_joint', 'arm_6_joint',
                         'arm_7_joint']

        reference_1.name = ['arm_2_joint', 'arm_3_joint']
        reference_2.name = ['arm_1_joint', 'arm_3_joint']
        reference_3.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                            'arm_4_joint', 'arm_5_joint', 'arm_6_joint',
                            'arm_7_joint']

        actual_1.position = [-1.4625, -1.4834, -2.5729]
        actual_2.position = [-1.4625, -1.4834, -2.5729]
        actual_3.position = [-1.4625, -1.4834, -2.5729, -2.0431,
                             0.4070, 1.8124, -2.8385]
        reference_1.position = [-2.5729, 0.0431, 4.2222]
        reference_2.position = [-2.5729, -0.0431, 4.2222]
        reference_3.position = [-1.4625, -0.4834, -2.5729, -2.0431,
                                0.4070, -1.8124, -0.8385]

        tolerance = 0.05
        result = False

        self.assertEqual(joint_position_monitors.check_joint_positions(
            actual_1.position, reference_1.position, tolerance), result)
        self.assertEqual(joint_position_monitors.check_joint_positions(
            actual_2.position, reference_2.position, tolerance), result)
        self.assertEqual(joint_position_monitors.check_joint_positions(
            actual_3.position, reference_3.position, tolerance), result)

    def test_check_positions_true(self):
        """
        Tests that the 'check_joint_positions' function returns True,
        since all joints have reached their desired position.

        """
        actual_1 = sensor_msgs.msg.JointState()
        actual_2 = sensor_msgs.msg.JointState()
        actual_3 = sensor_msgs.msg.JointState()

        reference_1 = sensor_msgs.msg.JointState()
        reference_2 = sensor_msgs.msg.JointState()
        reference_3 = sensor_msgs.msg.JointState()

        actual_1.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
        actual_2.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
        actual_3.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                         'arm_4_joint', 'arm_5_joint', 'arm_6_joint',
                         'arm_7_joint']

        reference_1.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
        reference_2.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
        reference_3.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                            'arm_4_joint', 'arm_5_joint', 'arm_6_joint',
                            'arm_7_joint']

        actual_1.position = [-1.4625, -1.4834, -2.5729]
        actual_2.position = [-1.4625, -1.4834, -2.5729]
        actual_3.position = [-1.4625, -1.4834, -2.5729, -2.0431,
                             0.4070, 1.8124, -2.8385]
        reference_1.position = [-1.4625, -1.4834, -2.5729]
        reference_2.position = [-1.4625, -1.4888, -2.5629]
        reference_3.position = [-1.4725, -1.4634, -2.5429, -2.0431,
                                0.4078, 1.8184, -2.8785]
        tolerance = 0.05
        result = True

        self.assertEqual(joint_position_monitors.check_joint_positions(
            actual_1.position, reference_1.position, tolerance), result)
        self.assertEqual(joint_position_monitors.check_joint_positions(
            actual_2.position, reference_2.position, tolerance), result)
        self.assertEqual(joint_position_monitors.check_joint_positions(
            actual_3.position, reference_3.position, tolerance), result)

    def test_insufficient_joint_positions(self):
        """
        Tests that the 'sort_joint_values' function raises an error,
        since the actual joint positions are lesser than the desired.

        """
        actual = sensor_msgs.msg.JointState()
        reference = sensor_msgs.msg.JointState()

        actual.name = ['arm_1_joint']
        reference.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']

        actual.position = [-1.4625]
        reference.position = [-1.4625, -1.4834, -2.5729]

        with self.assertRaises(AssertionError):
            joint_position_monitors.sort_joint_values(actual, reference)

    def test_sorting_joint_values(self):
        """
        Tests that the 'sort_joint_values' function returns a set
        of joint values matching the order of the 'actual' joint values.

        """
        actual = sensor_msgs.msg.JointState()
        reference = sensor_msgs.msg.JointState()

        # same order
        actual.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
        reference.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']

        actual.position = [1.0, 2.0, 3.0]
        reference.position = [4.0, 5.0, 6.0]

        desired = [4.0, 5.0, 6.0]

        result = joint_position_monitors.sort_joint_values(actual, reference)
        testing.assert_almost_equal(result, desired)

        # unsorted
        actual.name = ['arm_3_joint', 'arm_2_joint', 'arm_1_joint']
        reference.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']

        actual.position = [1.0, 2.0, 3.0]
        reference.position = [4.0, 5.0, 6.0]

        desired = [6.0, 5.0, 4.0]

        result = joint_position_monitors.sort_joint_values(actual, reference)
        testing.assert_almost_equal(result, desired)

        actual.name = ['arm_2_joint', 'arm_3_joint', 'arm_1_joint']
        reference.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']

        actual.position = [1.0, 2.0, 3.0]
        reference.position = [4.0, 5.0, 6.0]

        desired = [5.0, 6.0, 4.0]

        result = joint_position_monitors.sort_joint_values(actual, reference)
        testing.assert_almost_equal(result, desired)

    def test_unsorted_joint_values(self):
        """
        Tests that the 'check_joint_positions' function returns a correct result,
        even if the order of the 'actual' and 'reference' joint positions does not match.

        """
        tolerance = 0.05

        actual = sensor_msgs.msg.JointState()
        reference = sensor_msgs.msg.JointState()

        actual.name = ['arm_3_joint', 'arm_1_joint', 'arm_2_joint']
        actual.position = [-2.5729, -1.4688, -1.4834]
        reference.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
        reference.position = [-1.4625, -1.4834, -2.5729]

        sorted_references = joint_position_monitors.sort_joint_values(actual, reference)

        result = joint_position_monitors.check_joint_positions(
            actual.position, sorted_references, tolerance
        )
        self.assertEqual(result, True)

        tolerance = 0.00005
        result = joint_position_monitors.check_joint_positions(
            actual.position, sorted_references, tolerance
        )
        self.assertEqual(result, False)

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_joint_monitors', TestJointMonitors)
