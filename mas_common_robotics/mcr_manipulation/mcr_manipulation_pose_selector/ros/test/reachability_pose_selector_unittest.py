#!/usr/bin/env python
"""
Test unit for the functions in the reachability_pose_selector_utils.py module.

"""

PKG = 'mcr_manipulation_pose_selector'

import genpy
import copy
import unittest
import rosunit
import std_msgs.msg
import geometry_msgs.msg
import brics_actuator.msg
import mcr_manipulation_pose_selector_ros.reachability_pose_selector_utils as pose_selector_utils


class TestReachabilityPoseSelector(unittest.TestCase):
    """
    Tests functions used in the reachability_pose_selector_utils.py module.

    """
    def test_add_linear_offset_no_changes(self):
        """
        Tests that the 'add_linear_offset_to_pose' function returns the same pose,
        since no offset is specified.

        """
        pose_in = geometry_msgs.msg.PoseStamped()
        pose_in.pose.position.x = 1.0
        pose_in.pose.position.y = 2.0
        pose_in.pose.position.z = 3.0

        desired = copy.deepcopy(pose_in)
        actual = pose_selector_utils.add_linear_offset_to_pose(pose_in)

        self.assertEqual(actual, desired)

    def test_add_linear_offset_bad_input(self):
        """
        Tests that the 'add_linear_offset_to_pose' function raises an exception caused
        by specifying a wrong argument (offset).

        """
        pose_in = geometry_msgs.msg.PoseStamped()
        pose_in.pose.position.x = 1.0
        pose_in.pose.position.y = 2.0
        pose_in.pose.position.z = 3.0

        offset_1 = 3                # scalar instead of list
        offset_2 = [1.2, 3.8]       # list is too short
        offset_3 = [1, 2, 3, 4, 5]  # list is too long

        with self.assertRaises(AssertionError):
            pose_selector_utils.add_linear_offset_to_pose(pose_in, offset_1)
        with self.assertRaises(AssertionError):
            pose_selector_utils.add_linear_offset_to_pose(pose_in, offset_2)
        with self.assertRaises(AssertionError):
            pose_selector_utils.add_linear_offset_to_pose(pose_in, offset_3)

    def test_add_linear_offset_complete(self):
        """
        Tests that the 'add_linear_offset_to_pose' function returns a pose
        with the specified linear offset.

        """
        offset_1 = [3, 4, 5]
        offset_2 = [-3, 4, -5]
        offset_3 = [0, -8, 0]

        pose_in = geometry_msgs.msg.PoseStamped()
        pose_in.pose.position.x = 1.0
        pose_in.pose.position.y = 2.0
        pose_in.pose.position.z = 3.0

        desired_1 = geometry_msgs.msg.PoseStamped()
        desired_1.pose.position.x = 4.0
        desired_1.pose.position.y = 6.0
        desired_1.pose.position.z = 8.0

        desired_2 = geometry_msgs.msg.PoseStamped()
        desired_2.pose.position.x = -2.0
        desired_2.pose.position.y = 6.0
        desired_2.pose.position.z = -2.0

        desired_3 = geometry_msgs.msg.PoseStamped()
        desired_3.pose.position.x = 1.0
        desired_3.pose.position.y = -6.0
        desired_3.pose.position.z = 3.0

        actual = pose_selector_utils.add_linear_offset_to_pose(pose_in, offset_1)
        self.assertEqual(actual, desired_1)

        actual = pose_selector_utils.add_linear_offset_to_pose(pose_in, offset_2)
        self.assertEqual(actual, desired_2)

        actual = pose_selector_utils.add_linear_offset_to_pose(pose_in, offset_3)
        self.assertEqual(actual, desired_3)

    def test_list_to_brics_joints_bad_input(self):
        """
        Tests that the 'list_to_brics_joints' function raises an exception caused
        by specifying a wrong argument.

        """
        joint_values_bad = 5            # scalar instead of list
        joint_values_good = [1, 2, 3]
        joint_names_bad = ['a', 'b']    # list is too short
        joint_names_good = ['a', 'b', 'c']
        time_stamp_bad = 1.0            # float instead of std_msgs.msg.Time()

        with self.assertRaises(AssertionError):
            pose_selector_utils.list_to_brics_joints(joint_values_bad, joint_names_bad)
        with self.assertRaises(AssertionError):
            pose_selector_utils.list_to_brics_joints(joint_values_bad, joint_names_good)
        with self.assertRaises(AssertionError):
            pose_selector_utils.list_to_brics_joints(joint_values_good, joint_names_bad)
        with self.assertRaises(AssertionError):
            pose_selector_utils.list_to_brics_joints(
                joint_values_good, joint_names_good, time_stamp=time_stamp_bad
            )

    def test_list_to_brics_joints_basic(self):
        """
        Tests that the 'list_to_brics_joints' function returns a correct
        brics_actuator.msg.JointPositions, specifying only the required arguments.

        """
        joint_values = [4, 5, 8]
        joint_names = ['a', 'b', 'c']

        desired = brics_actuator.msg.JointPositions()
        desired.positions.append(brics_actuator.msg.JointValue(joint_uri='a', value=4))
        desired.positions.append(brics_actuator.msg.JointValue(joint_uri='b', value=5))
        desired.positions.append(brics_actuator.msg.JointValue(joint_uri='c', value=8))

        actual = pose_selector_utils.list_to_brics_joints(joint_values, joint_names)
        self.assertEqual(actual, desired)

    def test_list_to_brics_joints_complete(self):
        """
        Tests that the 'list_to_brics_joints' function returns a correct
        brics_actuator.msg.JointPositions, specifying all arguments.

        """
        joint_values = [4, 5, 8]
        joint_names = ['a', 'b', 'c']
        time_stamp = std_msgs.msg.Time(genpy.Time(1, 0))
        unit = 'radians'

        desired = brics_actuator.msg.JointPositions()
        desired.positions.append(
            brics_actuator.msg.JointValue(
                joint_uri='a', value=4, timeStamp=time_stamp, unit=unit)
        )
        desired.positions.append(
            brics_actuator.msg.JointValue(
                joint_uri='b', value=5, timeStamp=time_stamp, unit=unit)
        )
        desired.positions.append(
            brics_actuator.msg.JointValue(
                joint_uri='c', value=8, timeStamp=time_stamp, unit=unit)
        )

        actual = pose_selector_utils.list_to_brics_joints(
            joint_values, joint_names, time_stamp=time_stamp, unit=unit)
        self.assertEqual(actual, desired)

    def test_pose_array_to_list_bad_input(self):
        """
        Tests that the 'pose_array_to_list' function raises an exception caused
        by specifying a wrong argument.

        """
        pose_1 = 3          # scalar instead of geometry_msgs.msg.PoseArray
        pose_2 = [1, 2, 3]  # list instead of geometry_msgs.msg.PoseArray

        # geometry_msgs.msg.PoseStamped instead of geometry_msgs.msg.PoseArray
        pose_3 = geometry_msgs.msg.PoseStamped()
        pose_3.pose.position.x = 1.0
        pose_3.pose.position.y = 2.0
        pose_3.pose.position.z = 3.0

        with self.assertRaises(AssertionError):
            pose_selector_utils.pose_array_to_list(pose_1)
        with self.assertRaises(AssertionError):
            pose_selector_utils.pose_array_to_list(pose_2)
        with self.assertRaises(AssertionError):
            pose_selector_utils.pose_array_to_list(pose_3)

    def test_pose_array_to_list_basic(self):
        """
        Tests that the 'pose_array_to_list' function returns a list of
        geometry_msgs.msg.PoseStamped objects.

        """
        pose_header = std_msgs.msg.Header(frame_id="base_link")

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

        poses = geometry_msgs.msg.PoseArray(
            header=pose_header, poses=[pose_1, pose_2, pose_3]
        )

        desired = [
            geometry_msgs.msg.PoseStamped(header=pose_header, pose=pose_1),
            geometry_msgs.msg.PoseStamped(header=pose_header, pose=pose_2),
            geometry_msgs.msg.PoseStamped(header=pose_header, pose=pose_3)
        ]

        actual = pose_selector_utils.pose_array_to_list(poses)
        self.assertEqual(actual, desired)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_reachability_pose_selector', TestReachabilityPoseSelector)
