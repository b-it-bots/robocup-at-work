#!/usr/bin/env python
"""
Integration test for the 'twist_controller' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg

PKG = 'mcr_twist_controller'


class TestTwistController(unittest.TestCase):
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
        self.pose_error.unregister()

    def test_cartesian_distance_controller(self):
        """
        Verifies that the node returns correctly the Cartesian velocities,
        based on the Cartesian distances between two poses.

        """
        pose_error = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        pose_error.linear.x = 0.2
        pose_error.linear.y = 0.001
        pose_error.linear.z = 0.3
        pose_error.angular.x = 0.2
        pose_error.angular.y = 0.0
        pose_error.angular.z = -0.4

        expected_result = geometry_msgs.msg.TwistStamped()
        expected_result.twist.linear.x = 0.16
        expected_result.twist.linear.y = 0.0
        expected_result.twist.linear.z = -0.24
        expected_result.twist.angular.x = -0.16
        expected_result.twist.angular.y = 0.0
        expected_result.twist.angular.z = 0.32

        while not self.wait_for_result:
            self.pose_error.publish(pose_error)
            self.event_out.publish('e_start')

        self.assertAlmostEqual(
            self.result.twist.linear.x, expected_result.twist.linear.x, places=6
        )
        self.assertAlmostEqual(
            self.result.twist.linear.z, expected_result.twist.linear.z, places=6
        )
        self.assertAlmostEqual(
            self.result.twist.angular.x, expected_result.twist.angular.x, places=6
        )
        self.assertAlmostEqual(
            self.result.twist.angular.y, expected_result.twist.angular.y, places=6
        )
        self.assertAlmostEqual(
            self.result.twist.angular.z, expected_result.twist.angular.z, places=6
        )

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('twist_controller_test')
    rostest.rosrun(PKG, 'twist_controller_test', TestTwistController)
