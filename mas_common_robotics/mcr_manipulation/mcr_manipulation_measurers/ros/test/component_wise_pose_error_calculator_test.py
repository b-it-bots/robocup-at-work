#!/usr/bin/env python
"""
Test integration for the component_wise_pose_error_calculator node.

"""

import unittest
import rostest
import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg
import tf

PKG = 'mcr_manipulation_measurers'


class TestComponentWisePoseErrorCalculator(unittest.TestCase):
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
        self.pose_1 = rospy.Publisher(
            '~pose_1', geometry_msgs.msg.PoseStamped
        )
        self.pose_2 = rospy.Publisher(
            '~pose_2', geometry_msgs.msg.PoseStamped
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output',
            mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference,
            self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.pose_2.unregister()
        self.pose_1.unregister()

    def test_pose_error_calculator(self):
        """
        Verifies that the node returns correctly the error between
        the two Cartesian poses.

        """
        expected_result = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        expected_result.linear.x = 0.251
        expected_result.linear.y = -0.016
        expected_result.linear.z = 0.411
        expected_result.angular.x = 0.0
        expected_result.angular.y = 0.0
        expected_result.angular.z = 0.0

        current = geometry_msgs.msg.PoseStamped()
        current.header.frame_id = 'base_link'
        current.pose.position.x = 0.0
        current.pose.position.y = 0.0
        current.pose.position.z = 0.0
        current.pose.orientation.x = 0.0
        current.pose.orientation.y = 0.0
        current.pose.orientation.z = 0.0
        current.pose.orientation.w = 1.0

        target = geometry_msgs.msg.PoseStamped()
        target.header.frame_id = 'arm_link_5'
        target.pose.position.x = 0.0
        target.pose.position.y = 0.0
        target.pose.position.z = 0.0
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 1.0

        translation = (0.251, -0.016, 0.411)
        rotation = (0.0, 0.0, 0.0, 1.0)
        current_frame = 'base_link'
        target_frame = 'arm_link_5'

        broadcaster = tf.TransformBroadcaster()

        while not self.wait_for_result:
            broadcaster.sendTransform(
                translation, rotation, rospy.Time.now(), current_frame, target_frame
            )

            self.pose_2.publish(current)
            self.pose_1.publish(target)
            self.event_out.publish('e_start')

        self.assertAlmostEqual(expected_result.linear.x, self.result.linear.x)
        self.assertAlmostEqual(expected_result.linear.y, self.result.linear.y)
        self.assertAlmostEqual(expected_result.linear.z, self.result.linear.z)
        self.assertAlmostEqual(expected_result.angular.x, self.result.angular.x)
        self.assertAlmostEqual(expected_result.angular.y, self.result.angular.y)
        self.assertAlmostEqual(expected_result.angular.z, self.result.angular.z)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('component_wise_pose_error_calculator_test')
    rostest.rosrun(PKG, 'component_wise_pose_error_calculator_test', TestComponentWisePoseErrorCalculator)
