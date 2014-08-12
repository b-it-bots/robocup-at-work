#!/usr/bin/env python
"""
Integration test for the 'cartesian_motion' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg

PKG = 'mcr_arm_motions'


class TestCartesianMotion(unittest.TestCase):
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
        self.desired_velocity = rospy.Publisher(
            '~desired_velocity', geometry_msgs.msg.TwistStamped
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
        self.desired_velocity.unregister()

    def test_cartesian_motion(self):
        """
        Verifies that the node returns the correct twist message.

        """
        test_velocity = geometry_msgs.msg.TwistStamped()
        test_velocity.twist.linear.x = 0.0
        test_velocity.twist.linear.y = 0.0
        test_velocity.twist.linear.z = 1.0
        test_velocity.twist.angular.x = 0.0
        test_velocity.twist.angular.y = 0.0
        test_velocity.twist.angular.z = 0.0

        expected_result = geometry_msgs.msg.TwistStamped()
        expected_result.twist.linear.x = 0.0
        expected_result.twist.linear.y = 0.0
        expected_result.twist.linear.z = 1.0
        expected_result.twist.angular.x = 0.0
        expected_result.twist.angular.y = 0.0
        expected_result.twist.angular.z = 0.0

        while not self.wait_for_result:
            self.event_out.publish('e_start')
            self.desired_velocity.publish(test_velocity)

        self.assertAlmostEqual(
            self.result.twist.linear.x, expected_result.twist.linear.x
        )
        self.assertAlmostEqual(
            self.result.twist.linear.z, expected_result.twist.linear.z
        )

        # Check if the motion stops.
        self.wait_for_result = None

        while not self.wait_for_result:
            self.event_out.publish('e_stop')

        self.assertAlmostEqual(self.result.twist.linear.x, 0.0)
        self.assertAlmostEqual(self.result.twist.linear.y, 0.0)
        self.assertAlmostEqual(self.result.twist.linear.z, 0.0)
        self.assertAlmostEqual(self.result.twist.angular.x, 0.0)
        self.assertAlmostEqual(self.result.twist.angular.y, 0.0)
        self.assertAlmostEqual(self.result.twist.angular.z, 0.0)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('cartesian_motion_test')
    rostest.rosrun(PKG, 'cartesian_motion_test', TestCartesianMotion)
