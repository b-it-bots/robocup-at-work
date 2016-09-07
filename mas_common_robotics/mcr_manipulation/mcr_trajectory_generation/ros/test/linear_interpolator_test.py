#!/usr/bin/env python
"""
Integration test for the 'linear_interpolator' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg

PKG = 'mcr_trajectory_generation'


class TestLinearInterpolator(unittest.TestCase):
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
        self.start_pose = rospy.Publisher('~start_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.goal_pose = rospy.Publisher('~goal_pose', geometry_msgs.msg.PoseStamped, queue_size=1)

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', geometry_msgs.msg.PoseArray, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.start_pose.unregister()
        self.goal_pose.unregister()
        self.component_output.unregister()

    def test_configuration_crossover(self):
        """
        Verifies that the node returns a list of poses between the start and goal pose.

        """
        # Note: this should match what is specified in the .test file
        max_poses = 5
        start_pose = geometry_msgs.msg.PoseStamped()
        goal_pose = geometry_msgs.msg.PoseStamped()
        start_pose.header.frame_id = "base_link"
        goal_pose.header.frame_id = "base_link"

        start_pose.pose.position.x = 1.0
        start_pose.pose.position.y = 2.0
        start_pose.pose.position.z = 3.0
        start_pose.pose.orientation.w = 1.0

        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = 2.0
        goal_pose.pose.position.z = 3.0
        goal_pose.pose.orientation.w = 1.0

        while not self.wait_for_result:
            self.event_out.publish('e_start')
            self.start_pose.publish(start_pose)
            self.goal_pose.publish(goal_pose)

        self.assertEqual(len(self.result.poses), max_poses)

        # test that the middle pose is between the start and goal poses
        self.assertAlmostEqual(self.result.poses[2].position.x, 1.5)
        self.assertAlmostEqual(self.result.poses[2].position.y, 2.0)
        self.assertAlmostEqual(self.result.poses[2].position.z, 3.0)

        # test that the first and last poses are equal to the start and goal poses
        self.assertEqual(self.result.poses[0], start_pose.pose)
        self.assertEqual(self.result.poses[-1], goal_pose.pose)

        # test that the orientation of the interpolated poses is equal to the
        # orientation of the start pose
        self.assertEqual(self.result.poses[1].orientation, start_pose.pose.orientation)
        self.assertEqual(self.result.poses[2].orientation, start_pose.pose.orientation)
        self.assertEqual(self.result.poses[3].orientation, start_pose.pose.orientation)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('linear_interpolator_test')
    rostest.rosrun(PKG, 'linear_interpolator_test', TestLinearInterpolator)
