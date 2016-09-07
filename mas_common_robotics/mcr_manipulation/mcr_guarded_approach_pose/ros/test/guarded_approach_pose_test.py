#!/usr/bin/env python
"""
Integration test for the 'guarded_approach_pose' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

PKG = 'mcr_guarded_approach_pose'


class TestGuardedApproachPose(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.pub_current_joints = rospy.Publisher(
            '~current_joint_values', sensor_msgs.msg.JointState, queue_size=1
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', geometry_msgs.msg.TwistStamped, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.pub_current_joints.unregister()
        self.component_output.unregister()

    def test_guarded_approach_pose(self):
        """
        Verifies that the node is able to configure and start the required components,
        i.e. not a functional test.

        """
        # Note: This has to match the .test file
        desired_twist = geometry_msgs.msg.TwistStamped()
        desired_twist.header.frame_id = 'arm_link_5'
        desired_twist.twist.linear.z = 1.0

        current_joints = sensor_msgs.msg.JointState()
        current_joints.name = ['arm_joint_2', 'arm_joint_3', 'arm_joint_4']
        current_joints.position = [0.2, 1.8, 1.7]
        current_joints.effort = [0.2, 1.8, 1.7]

        while not self.wait_for_result:
            self.pub_current_joints.publish(current_joints)
            self.event_out.publish('e_start')

        self.assertEqual(self.result.header.frame_id, desired_twist.header.frame_id)
        self.assertEqual(
            self.result.twist, desired_twist.twist,
            msg="Result: {0}\nDesired: {1}".format(self.result, desired_twist)
        )

        self.result = None
        self.wait_for_result = None

        while not self.wait_for_result:
            self.event_out.publish('e_collision')

        # the twist should be zero after a collision is detected
        desired_twist.twist.linear.z = 0.0
        self.assertEqual(
            self.result.twist, desired_twist.twist,
            msg="Result: {0}\nDesired: {1}".format(self.result, desired_twist)
        )

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('guarded_approach_pose_test')
    rostest.rosrun(PKG, 'guarded_approach_pose_test', TestGuardedApproachPose)
