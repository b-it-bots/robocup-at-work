#!/usr/bin/env python
"""
Test unit for the functions used in ik_trajectory_solver.py module.

"""

PKG = 'mcr_trajectory_generation'

import unittest
import rosunit
import mcr_trajectory_generation_ros.ik_trajectory_solver_utils as utils
import trajectory_msgs.msg
import geometry_msgs.msg


class TestIkTrajectorySolver(unittest.TestCase):
    """
    Tests the functions used in the ik_trajectory_solver.py module.

    """
    def test_list_to_joint_trajectory_bad_input(self):
        """
        Tests that the 'list_to_joint_trajectory' function raises an exception caused
        by specifying a wrong argument.

        """
        bad_configurations = [5, 4]   # it should be a list of lists
        good_joint_values = [[1, 2, 3]]
        bad_joint_names = ['a', 'b']        # list is too short
        good_joint_names = ['a', 'b', 'c']
        bad_time_stamp = 1.0                # float instead of std_msgs.msg.Time()

        with self.assertRaises(AssertionError):
            utils.list_to_joint_trajectory(bad_configurations, bad_joint_names)
        with self.assertRaises(AssertionError):
            utils.list_to_joint_trajectory(bad_configurations, good_joint_names)
        with self.assertRaises(AssertionError):
            utils.list_to_joint_trajectory(good_joint_values, bad_joint_names)
        with self.assertRaises(AssertionError):
            utils.list_to_joint_trajectory(
                good_joint_values, good_joint_names, time_stamp=bad_time_stamp
            )

    def test_list_to_joint_trajectory(self):
        """
        Tests that the 'list_to_joint_trajectory' returns
        a correct JointTrajectory message.

        """
        joint_configurations = [
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
            [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0],
            [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
            [4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
            [5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0]
        ]
        joint_names = ['a', 'b', 'c', 'd', 'e', 'f', 'g']
        reference_frame = "base_link"

        point_1 = trajectory_msgs.msg.JointTrajectoryPoint(
            positions=joint_configurations[0]
        )
        point_2 = trajectory_msgs.msg.JointTrajectoryPoint(
            positions=joint_configurations[1]
        )
        point_3 = trajectory_msgs.msg.JointTrajectoryPoint(
            positions=joint_configurations[2]
        )
        point_4 = trajectory_msgs.msg.JointTrajectoryPoint(
            positions=joint_configurations[3]
        )
        point_5 = trajectory_msgs.msg.JointTrajectoryPoint(
            positions=joint_configurations[4]
        )

        desired = trajectory_msgs.msg.JointTrajectory()
        desired.header.frame_id = reference_frame
        desired.joint_names = joint_names
        desired.points.extend([point_1, point_2, point_3, point_4, point_5])

        actual = utils.list_to_joint_trajectory(
            joint_configurations, joint_names, frame_id=reference_frame
        )
        self.assertEqual(actual, desired)

    def test_limit_number_of_poses_basic(self):
        """
        Tests that the 'limit_number_of_poses' returns
        a correct amount of poses.

        """
        pose_1 = geometry_msgs.msg.Pose()
        pose_1.position.x = 1.0
        pose_1.position.y = 2.0
        pose_1.position.z = 3.0

        pose_2 = geometry_msgs.msg.Pose()
        pose_2.position.x = 4.0
        pose_2.position.y = 5.0
        pose_2.position.z = 6.0

        pose_3 = geometry_msgs.msg.Pose()
        pose_3.position.x = 7.0
        pose_3.position.y = 8.0
        pose_3.position.z = 9.0

        pose_4 = geometry_msgs.msg.Pose()
        pose_4.position.x = 10.0
        pose_4.position.y = 11.0
        pose_4.position.z = 12.0

        pose_5 = geometry_msgs.msg.Pose()
        pose_5.position.x = 13.0
        pose_5.position.y = 14.0
        pose_5.position.z = 15.0

        poses = geometry_msgs.msg.PoseArray(
            poses=[pose_1, pose_2, pose_3, pose_4, pose_5]
        )

        actual = utils.limit_number_of_poses(poses, max_poses=2)
        self.assertEqual(len(actual.poses), 2)

        actual = utils.limit_number_of_poses(poses, max_poses=3)
        self.assertEqual(len(actual.poses), 3)

        actual = utils.limit_number_of_poses(poses, max_poses=4)
        self.assertEqual(len(actual.poses), 4)

        actual = utils.limit_number_of_poses(poses, max_poses=5)
        self.assertEqual(len(actual.poses), 5)

        actual = utils.limit_number_of_poses(poses)
        self.assertEqual(len(actual.poses), 5)

    def test_limit_number_of_poses_complete(self):
        """
        Tests that the 'limit_number_of_poses' returns the correct amount
        of poses and those poses should be equally spaced.

        """
        pose_1 = geometry_msgs.msg.Pose()
        pose_1.position.x = 1.0
        pose_1.position.y = 2.0
        pose_1.position.z = 3.0

        pose_2 = geometry_msgs.msg.Pose()
        pose_2.position.x = 4.0
        pose_2.position.y = 5.0
        pose_2.position.z = 6.0

        pose_3 = geometry_msgs.msg.Pose()
        pose_3.position.x = 7.0
        pose_3.position.y = 8.0
        pose_3.position.z = 9.0

        pose_4 = geometry_msgs.msg.Pose()
        pose_4.position.x = 10.0
        pose_4.position.y = 11.0
        pose_4.position.z = 12.0

        pose_5 = geometry_msgs.msg.Pose()
        pose_5.position.x = 13.0
        pose_5.position.y = 14.0
        pose_5.position.z = 15.0

        pose_6 = geometry_msgs.msg.Pose()
        pose_6.position.x = 16.0
        pose_6.position.y = 17.0
        pose_6.position.z = 18.0

        pose_7 = geometry_msgs.msg.Pose()
        pose_7.position.x = 19.0
        pose_7.position.y = 20.0
        pose_7.position.z = 21.0

        pose_8 = geometry_msgs.msg.Pose()
        pose_8.position.x = 22.0
        pose_8.position.y = 23.0
        pose_8.position.z = 24.0

        poses = geometry_msgs.msg.PoseArray(
            poses=[pose_1, pose_2, pose_3, pose_4, pose_5, pose_6, pose_7, pose_8]
        )

        actual = utils.limit_number_of_poses(poses, 2)
        desired = geometry_msgs.msg.PoseArray(poses=[pose_1, pose_8])
        self.assertEqual(actual, desired)

        actual = utils.limit_number_of_poses(poses, 3)
        desired = geometry_msgs.msg.PoseArray(poses=[pose_1, pose_4, pose_8])
        self.assertEqual(actual, desired)

        actual = utils.limit_number_of_poses(poses, 4)
        desired = geometry_msgs.msg.PoseArray(poses=[pose_1, pose_4, pose_6, pose_8])
        self.assertEqual(actual, desired)

    def test_limit_number_of_poses_bad_input(self):
        """
        Tests that the 'limit_number_of_poses' function raises an
        exception caused by specifying a wrong argument.

        """
        pose_1 = geometry_msgs.msg.Pose()
        pose_1.position.x = 1.0
        pose_1.position.y = 2.0
        pose_1.position.z = 3.0

        pose_2 = geometry_msgs.msg.Pose()
        pose_2.position.x = 4.0
        pose_2.position.y = 5.0
        pose_2.position.z = 6.0

        pose_3 = geometry_msgs.msg.Pose()
        pose_3.position.x = 5.0
        pose_3.position.y = 8.0
        pose_3.position.z = 9.0

        pose_4 = geometry_msgs.msg.Pose()
        pose_4.position.x = 5.0
        pose_4.position.y = 8.0
        pose_4.position.z = 9.0

        pose_5 = geometry_msgs.msg.Pose()
        pose_5.position.x = 5.0
        pose_5.position.y = 8.0
        pose_5.position.z = 9.0

        bad_poses_1 = 50.4          # float instead of geometry_msgs.msg.PoseStamped
        bad_poses_2 = [5, 4]        # list instead of geometry_msgs.msg.PoseStamped
        good_poses = geometry_msgs.msg.PoseArray(
            poses=[pose_1, pose_2, pose_3, pose_4, pose_5]
        )

        with self.assertRaises(AssertionError):
            utils.limit_number_of_poses(bad_poses_1)
        with self.assertRaises(AssertionError):
            utils.limit_number_of_poses(bad_poses_2)
        with self.assertRaises(AssertionError):
            utils.limit_number_of_poses(good_poses, -1)
        with self.assertRaises(AssertionError):
            utils.limit_number_of_poses(good_poses, 0)
        with self.assertRaises(AssertionError):
            utils.limit_number_of_poses(good_poses, 1)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_ik_trajectory_solver', TestIkTrajectorySolver)
