#!/usr/bin/env python
"""
Integration test for the 'joint_position_monitor' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import tf

PKG = 'mcr_manipulation_converters'


class TestJointPositionMonitor(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True
        )
        self.desired_joint_positions = rospy.Publisher(
            '~desired_joint_positions', sensor_msgs.msg.JointState
        )
        self.joint_states = rospy.Publisher(
            '~joint_states', sensor_msgs.msg.JointState
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', std_msgs.msg.String, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.desired_joint_positions.unregister()
        self.joint_states.unregister()
        self.event_out.unregister()

    def test_joint_position_monitor(self):
        """
        Verifies that the node returns a correct event,
        when the desired joint positions have been reached.

        """
        expected_result = std_msgs.msg.String()
        expected_result.data = 'e_done'

        desired_joint_positions = sensor_msgs.msg.JointState()
        current_joint_positions = sensor_msgs.msg.JointState()

        current_joint_positions.name = ['a', 'b', 'c', 'd']
        current_joint_positions.position = [-1.46, -1.48, -2.57, 0.52]

        desired_joint_positions.name = ['d', 'b']
        desired_joint_positions.position = [0.52, -1.48]

        while not self.wait_for_result:
            self.desired_joint_positions.publish(desired_joint_positions)
            self.joint_states.publish(current_joint_positions)
            self.event_out.publish('e_start')

        self.assertAlmostEqual(
            self.result.data, expected_result.data
        )

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('joint_position_monitor_test')
    rostest.rosrun(PKG, 'joint_position_monitor_test', TestJointPositionMonitor)
