#!/usr/bin/env python
"""
Integration test for the 'joint_distance_measurer' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import sensor_msgs.msg
import mcr_manipulation_msgs.msg

PKG = 'mcr_manipulation_measurers'


class TestJointDistanceMeasurer(unittest.TestCase):
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
        self.pub_current_joints = rospy.Publisher(
            '~current_joint_values', sensor_msgs.msg.JointState
        )
        self.pub_target_joints = rospy.Publisher(
            '~target_joint_values', sensor_msgs.msg.JointState
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', mcr_manipulation_msgs.msg.JointDistance,
            self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.pub_current_joints.unregister()
        self.pub_target_joints.unregister()

    def test_joint_distance_measurer(self):
        """
        Verifies that the node returns correctly the distances between
        the current and target joint positions.

        """
        current_joints = sensor_msgs.msg.JointState()
        current_joints.name = ['a', 'b', 'c', 'd', 'e', 'f']
        current_joints.position = [0.2, 1.8, 2.4, 0.6, 2.5, 1.7]

        target_joints = sensor_msgs.msg.JointState()
        target_joints.name = ['b', 'd', 'f']
        target_joints.position = [0.2, 1.8, 1.7]

        expected_result = mcr_manipulation_msgs.msg.JointDistance()
        expected_result.distance = [-1.6, 1.2, 0.0]
        expected_result.name = ['b', 'd', 'f']

        while not self.wait_for_result:
            self.pub_target_joints.publish(target_joints)
            self.pub_current_joints.publish(current_joints)
            self.event_out.publish('e_start')

        for i, result in enumerate(self.result.distance):
                self.assertAlmostEqual(result, expected_result.distance[i])

        self.assertEqual(self.result.name, expected_result.name)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('joint_distance_measurer_test')
    rostest.rosrun(PKG, 'joint_distance_measurer_test', TestJointDistanceMeasurer)
