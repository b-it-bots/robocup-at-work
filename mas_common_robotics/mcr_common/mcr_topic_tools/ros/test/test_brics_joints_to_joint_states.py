#!/usr/bin/env python
"""
Integration test for the 'brics_joints_to_joint_states' node.

"""

import unittest
import rospy
import rostest
import sensor_msgs.msg
import brics_actuator.msg

PKG = 'mcr_topic_tools'


class TestBricsJointsToJointStates(unittest.TestCase):
    """
    Tests the functions in the transform_topic_utils.py module.

    """
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.pub_joint_configuration = rospy.Publisher(
            '~joint_configuration', brics_actuator.msg.JointPositions, queue_size=1
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', sensor_msgs.msg.JointState, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.pub_joint_configuration.unregister()
        self.component_output.unregister()

    def test_brics_joints_to_joint_state(self):
        """
        Verifies that the node transforms a brics_actuator.msg.JointPositions message
        into a sensor_msgs.msg.JointState message.

        """
        joint_names = ['a', 'b', 'c', 'd']
        joint_positions = (1.0, 2.0, 3.0, 4.0)
        time_stamp = rospy.Time(1.0)

        message_in = brics_actuator.msg.JointPositions()
        for _ in joint_names:
            message_in.positions.append(brics_actuator.msg.JointValue())

        for joint, name, position in zip(
                message_in.positions, joint_names, joint_positions
        ):
            joint.timeStamp = time_stamp
            joint.joint_uri = name
            joint.value = position

        desired_message = sensor_msgs.msg.JointState()
        desired_message.header.stamp = time_stamp
        desired_message.name = joint_names
        desired_message.position = joint_positions

        while not self.wait_for_result:
            self.pub_joint_configuration.publish(message_in)

        self.assertEqual(self.result.header.stamp, desired_message.header.stamp)
        self.assertEqual(self.result.name, desired_message.name)
        self.assertEqual(self.result.position, desired_message.position)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('test_brics_joints_to_joint_states')
    rostest.rosrun(PKG, 'test_brics_joints_to_joint_states', TestBricsJointsToJointStates)
