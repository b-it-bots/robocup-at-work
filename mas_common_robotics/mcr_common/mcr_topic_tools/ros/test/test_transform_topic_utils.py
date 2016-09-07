#!/usr/bin/env python
"""
Test unit for the functions in the transform_topic_utils.py module.

"""

import unittest
import rospy
import rosunit
import sensor_msgs.msg
import brics_actuator.msg
import mcr_topic_tools_ros.transformer_utils as topic_utils

PKG = 'mcr_topic_tools'


class TestTransformTopicUtils(unittest.TestCase):
    """
    Tests the functions in the transform_topic_utils.py module.

    """
    def test_brics_joints_to_joint_state(self):
        """
        Tests that the 'brics_joints_to_joint_state' function correctly
        transforms a 'brics_actuator/JointPositions' message into a
        'sensor_msgs/JointState' message.

        """
        joint_names = ['a', 'b', 'c', 'd']
        joint_positions = [1, 2, 3, 4]
        # Abusing Duration as Time
        time_stamp = rospy.Duration(1.0)

        message_in = brics_actuator.msg.JointPositions()
        for _ in joint_names:
            message_in.positions.append(brics_actuator.msg.JointValue())

        for joint, name, position in zip(
                message_in.positions, joint_names, joint_positions
        ):
            joint.timeStamp = time_stamp
            joint.joint_uri = name
            joint.value = position

        message_out = sensor_msgs.msg.JointState()
        message_out.header.stamp = time_stamp
        message_out.name = joint_names
        message_out.position = joint_positions

        result = topic_utils.brics_joints_to_joint_state(message_in)

        self.assertEqual(result.header, message_out.header)
        self.assertEqual(result.name, message_out.name)
        self.assertEqual(result.position, message_out.position)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_transform_topic_utils', TestTransformTopicUtils)
