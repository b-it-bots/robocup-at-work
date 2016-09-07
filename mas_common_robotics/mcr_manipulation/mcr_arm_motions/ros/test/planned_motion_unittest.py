#!/usr/bin/env python
"""
Test unit for the functions in the planned_motion_utils.py module.

"""

PKG = 'mcr_arm_motions'

import unittest
import numpy.testing
import rosunit
import brics_actuator.msg
import mcr_arm_motions_ros.planned_motion_utils as planned_motion


class TestPlannedMotion(unittest.TestCase):
    """
    Tests methods used in the planned_motion.py module.

    """
    def test_brics_joint_positions_to_list(self):
        """
        Tests that the 'brics_joint_positions_to_list' function returns
        the correct list of joints.

        """
        joints_in = brics_actuator.msg.JointPositions()
        joints_in_names = ['a', 'b', 'c']
        joints_in_positions = [1.0, 2.0, 3.0]

        for _ in joints_in_names:
            joints_in.positions.append(brics_actuator.msg.JointValue())

        for joint, name, position in zip(
                joints_in.positions, joints_in_names, joints_in_positions
        ):
            joint.joint_uri = name
            joint.value = position

        desired = [1.0, 2.0, 3.0]
        actual = planned_motion.brics_joint_positions_to_list(joints_in)

        numpy.testing.assert_almost_equal(actual, desired)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_planned_motion', TestPlannedMotion)
