#!/usr/bin/env python
"""
Test unit for the functions used in the linear_interpolator_utils.py module.

"""

PKG = 'mcr_trajectory_generation'

import unittest
import rosunit
import geometry_msgs.msg
import mcr_trajectory_generation_ros.linear_interpolator_utils as utils


class TestLinearInterpolator(unittest.TestCase):
    """
    Tests the functions used in the linear_interpolator_utils.py module.

    """
    def test_interpolate_point_bad_input(self):
        """
        Tests that the 'interpolate_point' function raises an exception caused
        by specifying a wrong argument.

        """
        good_point_1 = [1, 2, 3]
        bad_point_1 = 5.5                 # it should be a list
        good_point_2 = [1, 1, 1]
        bad_point_2 = [5, 2]             # list too short
        good_distance = 1.0
        bad_distance = [5, 2]           # it should be a float

        with self.assertRaises(AssertionError):
            utils.interpolate_point(good_point_1, good_point_2, bad_distance)
        with self.assertRaises(AssertionError):
            utils.interpolate_point(good_point_1, bad_point_2, good_distance)
        with self.assertRaises(AssertionError):
            utils.interpolate_point(bad_point_1, good_point_2, good_distance)
        with self.assertRaises(AssertionError):
            utils.interpolate_point(bad_point_1, bad_point_2, good_distance)

    def test_interpolate_point(self):
        """
        Tests that the 'interpolate_point' function returns the correct result.

        """
        point_1 = [0, 2, 3]
        point_2 = [1, 2, 3]
        distance = 0.1
        expected = [0.1, 2, 3]

        actual = utils.interpolate_point(point_1, point_2, distance)
        self.assertEqual(actual, expected)

    def test_interpolate_poses_bad_input(self):
        """
        Tests that the 'interpolate_poses' function raises an exception caused
        by specifying a wrong argument.

        """
        good_pose_1 = geometry_msgs.msg.PoseStamped()
        bad_pose_1 = geometry_msgs.msg.Pose()   # it should be a PoseStamped
        good_pose_2 = geometry_msgs.msg.PoseStamped()
        bad_pose_2 = [5, 2]             # it should be a PoseStamped
        good_max_poses = 5
        bad_max_poses = [5, 2]           # it should be an int

        with self.assertRaises(AssertionError):
            utils.interpolate_poses(good_pose_1, good_pose_2, bad_max_poses)
        with self.assertRaises(AssertionError):
            utils.interpolate_poses(good_pose_1, bad_pose_2, good_max_poses)
        with self.assertRaises(AssertionError):
            utils.interpolate_poses(bad_pose_1, good_pose_2, good_max_poses)
        with self.assertRaises(AssertionError):
            utils.interpolate_poses(bad_pose_1, bad_pose_2, good_max_poses)

    def test_interpolate_poses(self):
        """
        Tests that the 'interpolate_poses' function returns the correct result.

        """
        max_poses = 3

        pose_1 = geometry_msgs.msg.PoseStamped()
        pose_1.pose.position.x = 0.0
        pose_1.pose.position.y = 2.0
        pose_1.pose.position.z = 3.0
        pose_1.pose.orientation.w = 1.0

        pose_2 = geometry_msgs.msg.PoseStamped()
        pose_2.pose.position.x = 1.0
        pose_2.pose.position.y = 2.0
        pose_2.pose.position.z = 3.0
        pose_2.pose.orientation.w = 1.0

        p_1 = geometry_msgs.msg.Pose()
        p_1.position.x = 0.25
        p_1.position.y = 2.0
        p_1.position.z = 3.0
        p_1.orientation.w = 1.0

        p_2 = geometry_msgs.msg.Pose()
        p_2.position.x = 0.5
        p_2.position.y = 2.0
        p_2.position.z = 3.0
        p_2.orientation.w = 1.0

        p_3 = geometry_msgs.msg.Pose()
        p_3.position.x = 0.75
        p_3.position.y = 2.0
        p_3.position.z = 3.0
        p_3.orientation.w = 1.0

        expected = [p_1, p_2, p_3]

        actual = utils.interpolate_poses(pose_1, pose_2, max_poses)
        self.assertEqual(actual, expected)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_linear_interpolator', TestLinearInterpolator)
