#!/usr/bin/env python
"""
Integration test for the 'distance_constrained' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg

PKG = 'mcr_guarded_approach_pose'


class TestDistanceConstrained(unittest.TestCase):
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
        self.pose_error = rospy.Publisher(
            '~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference,
            latch=True, queue_size=1
        )
        self.desired_twist = rospy.Publisher(
            '~desired_twist', geometry_msgs.msg.TwistStamped, queue_size=1
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
        self.pose_error.unregister()
        self.desired_twist.unregister()
        self.component_output.unregister()

    def test_distance_constrained(self):
        """
        Verifies that the node is able to configure and start the required components,
        i.e. not a functional test.

        """
        pose_error = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        pose_error.header.frame_id = 'base_link'
        pose_error.linear.z = 1.0

        twist_in = geometry_msgs.msg.TwistStamped()
        twist_in.twist.linear.z = 1.0

        while not self.wait_for_result:
            self.pose_error.publish(pose_error)
            self.desired_twist.publish(twist_in)
            self.event_out.publish('e_start')

        self.assertAlmostEqual(self.result.twist.linear.z, 1.0)

        #ToDo: test if stopping works...

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('distance_constrained_test')
    rostest.rosrun(PKG, 'distance_constrained_test', TestDistanceConstrained)
