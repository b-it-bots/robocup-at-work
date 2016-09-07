#!/usr/bin/env python
"""
Integration test for the 'twist_synchronizer' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg

PKG = 'mcr_twist_synchronizer'


class TestTwistSynchronizer(unittest.TestCase):
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
        self.pose_error = rospy.Publisher(
            '~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference
        )
        self.twist = rospy.Publisher('~twist', geometry_msgs.msg.TwistStamped)

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', geometry_msgs.msg.TwistStamped, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.twist.unregister()
        self.pose_error.unregister()

    def test_twist_synchronizer(self):
        """
        Verifies that the node returns correctly the synchronized twist.

        """
        pose_error = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        pose_error.linear.x = 0.2
        pose_error.linear.y = 0.3
        pose_error.linear.z = 0.0
        pose_error.angular.x = 0.0
        pose_error.angular.y = 0.0
        pose_error.angular.z = 0.5

        twist = geometry_msgs.msg.TwistStamped()
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = 0.08
        twist.twist.linear.y = 1.0
        twist.twist.linear.z = 0.2
        twist.twist.angular.x = -0.4
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.7

        expected_result = geometry_msgs.msg.TwistStamped()
        expected_result.header.frame_id = "base_link"
        expected_result.twist.linear.x = 0.08
        expected_result.twist.linear.y = 0.12
        expected_result.twist.linear.z = 0.0
        expected_result.twist.angular.x = 0.0
        expected_result.twist.angular.y = 0.0
        expected_result.twist.angular.z = 0.2

        while not self.wait_for_result:
            self.twist.publish(twist)
            self.pose_error.publish(pose_error)
            self.event_out.publish('e_start')

        self.assertAlmostEqual(
            self.result.twist.linear.x, expected_result.twist.linear.x, places=4
        )
        self.assertAlmostEqual(
            self.result.twist.linear.y, expected_result.twist.linear.y, places=4
        )
        self.assertAlmostEqual(
            self.result.twist.linear.z, expected_result.twist.linear.z, places=4
        )
        self.assertAlmostEqual(
            self.result.twist.angular.x, expected_result.twist.angular.x, places=4
        )
        self.assertAlmostEqual(
            self.result.twist.angular.y, expected_result.twist.angular.y, places=4
        )
        self.assertAlmostEqual(
            self.result.twist.angular.z, expected_result.twist.angular.z, places=4
        )

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('twist_synchronizer_test')
    rostest.rosrun(PKG, 'twist_synchronizer_test', TestTwistSynchronizer)
