#!/usr/bin/env python
"""
Test unit for the functions used in dmp_based_task_space_lame.py module.

"""

PKG = 'mcr_trajectory_generation'

import numpy.testing as testing
import unittest
import rosunit
import mcr_trajectory_generation_ros.dmp_based_task_space as dmp_based_task_space
import geometry_msgs.msg
import dmp.msg
import dmp.srv


class TestDmpBasedTaskSpace(unittest.TestCase):
    """
    Tests the functions used in the dmp_based_task_space_lame.py module.

    """
    def test_dmp_plan_to_pose_array(self):
        """
        Tests that the 'dmp_plan_to_pose_array' returns
        a correct PoseArray message.

        """
        positions = [
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
            [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0],
            [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
            [4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
            [5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0]
        ]

        dmp_plan = dmp.srv.GetDMPPlanResponse()
        dmp_plan.plan.points = [dmp.msg.DMPPoint() for _ in positions]
        dmp_plan.plan.times = range(len(positions))
        for dmp_point, pos in zip(dmp_plan.plan.points, positions):
            dmp_point.positions = pos

        pose_1 = geometry_msgs.msg.Pose()
        pose_1.position.x = 1.0
        pose_1.position.y = 2.0
        pose_1.position.z = 3.0
        pose_1.orientation.x = 4.0
        pose_1.orientation.y = 5.0
        pose_1.orientation.z = 6.0
        pose_1.orientation.w = 7.0

        pose_2 = geometry_msgs.msg.Pose()
        pose_2.position.x = 2.0
        pose_2.position.y = 3.0
        pose_2.position.z = 4.0
        pose_2.orientation.x = 5.0
        pose_2.orientation.y = 6.0
        pose_2.orientation.z = 7.0
        pose_2.orientation.w = 8.0

        pose_3 = geometry_msgs.msg.Pose()
        pose_3.position.x = 3.0
        pose_3.position.y = 4.0
        pose_3.position.z = 5.0
        pose_3.orientation.x = 6.0
        pose_3.orientation.y = 7.0
        pose_3.orientation.z = 8.0
        pose_3.orientation.w = 9.0

        pose_4 = geometry_msgs.msg.Pose()
        pose_4.position.x = 4.0
        pose_4.position.y = 5.0
        pose_4.position.z = 6.0
        pose_4.orientation.x = 7.0
        pose_4.orientation.y = 8.0
        pose_4.orientation.z = 9.0
        pose_4.orientation.w = 10.0

        pose_5 = geometry_msgs.msg.Pose()
        pose_5.position.x = 5.0
        pose_5.position.y = 6.0
        pose_5.position.z = 7.0
        pose_5.orientation.x = 8.0
        pose_5.orientation.y = 9.0
        pose_5.orientation.z = 10.0
        pose_5.orientation.w = 11.0

        desired = geometry_msgs.msg.PoseArray()
        desired.poses = [pose_1, pose_2, pose_3, pose_4, pose_5]

        actual = dmp_based_task_space.dmp_plan_to_pose_array(dmp_plan)
        self.assertEqual(actual, desired)

    def test_dmp_plan_to_pose_array_bad_input(self):
        """
        Tests that the 'dmp_plan_to_pose_array' function raises an exception caused
        by specifying a wrong argument.

        """
        plan_1 = 3          # scalar instead of dmp.srv.GetDMPPlanResponse
        plan_2 = [1, 2, 3]  # list instead of dmp.srv.GetDMPPlanResponse

        # geometry_msgs.msg.PoseStamped instead of dmp.srv.GetDMPPlanResponse
        plan_3 = geometry_msgs.msg.PoseStamped()
        plan_3.pose.position.x = 1.0
        plan_3.pose.position.y = 2.0
        plan_3.pose.position.z = 3.0

        with self.assertRaises(AssertionError):
            dmp_based_task_space.dmp_plan_to_pose_array(plan_1)
        with self.assertRaises(AssertionError):
            dmp_based_task_space.dmp_plan_to_pose_array(plan_2)
        with self.assertRaises(AssertionError):
            dmp_based_task_space.dmp_plan_to_pose_array(plan_3)

    def test_pose_stamped_to_list(self):
        """
        Tests that the 'pose_stamped_to_list' returns a list with the
        values of a the specified pose.

        """
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 3.0
        pose.pose.orientation.x = 4.0
        pose.pose.orientation.y = 5.0
        pose.pose.orientation.z = 6.0
        pose.pose.orientation.w = 7.0

        actual = dmp_based_task_space.pose_stamped_to_list(pose)
        desired = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]

        testing.assert_almost_equal(actual, desired)

    def test_pose_stamped_to_list_bad_input(self):
        """
        Tests that the 'pose_stamped_to_list' function raises an exception caused
        by specifying a wrong argument.

        """
        pose_1 = 3          # scalar instead of geometry_msgs.msg.PoseStamped
        pose_2 = [1, 2, 3]  # list instead of geometry_msgs.msg.PoseStamped

        # geometry_msgs.msg.Pose instead of geometry_msgs.msg.PoseStamped
        pose_3 = geometry_msgs.msg.Pose()
        pose_3.position.x = 1.0
        pose_3.position.y = 2.0
        pose_3.position.z = 3.0

        with self.assertRaises(AssertionError):
            dmp_based_task_space.pose_stamped_to_list(pose_1)
        with self.assertRaises(AssertionError):
            dmp_based_task_space.pose_stamped_to_list(pose_2)
        with self.assertRaises(AssertionError):
            dmp_based_task_space.pose_stamped_to_list(pose_3)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_dmp_based_task_space', TestDmpBasedTaskSpace)
