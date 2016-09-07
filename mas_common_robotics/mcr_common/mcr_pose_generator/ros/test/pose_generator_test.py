#!/usr/bin/env python
"""
Test integration for the pose_generator node.

"""

import unittest
import rostest
import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg

PKG = 'mcr_pose_generator'


class TestPoseGenerator(unittest.TestCase):
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
        self.target_pose = rospy.Publisher(
            '~target_pose', geometry_msgs.msg.PoseStamped
        )
        self.sampling_parameters = rospy.Publisher(
            '~sampling_parameters', mcr_manipulation_msgs.msg.SphericalSamplerParameters
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', geometry_msgs.msg.PoseArray,
            self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.sampling_parameters.unregister()
        self.target_pose.unregister()

    def test_pose_generator(self):
        """
        Verifies that the node returns correctly a list of poses,
        according to a target pose and spherical sampler parameters.

        """
        expected_result = geometry_msgs.msg.PoseArray()
        expected_result.header.frame_id = 'base_link'

        pose_1 = geometry_msgs.msg.Pose()
        pose_1.position.x = 0.4
        pose_1.position.y = 0.0
        pose_1.position.z = 0.2
        pose_1.orientation.x = 0.0
        pose_1.orientation.y = 0.0
        pose_1.orientation.z = 0.0
        pose_1.orientation.w = 1.0

        pose_2 = geometry_msgs.msg.Pose()
        pose_2.position.x = 0.39
        pose_2.position.y = 0.0
        pose_2.position.z = 0.2
        pose_2.orientation.x = 0.0
        pose_2.orientation.y = 0.0
        pose_2.orientation.z = 0.0
        pose_2.orientation.w = 1.0

        expected_result.poses.append(pose_1)
        expected_result.poses.append(pose_2)

        sampling_params = mcr_manipulation_msgs.msg.SphericalSamplerParameters()
        sampling_params.height.minimum = 0.0
        sampling_params.height.maximum = 0.0
        sampling_params.zenith.minimum = 0.0
        sampling_params.zenith.maximum = 0.0
        sampling_params.azimuth.minimum = 0.0
        sampling_params.azimuth.maximum = 0.0
        sampling_params.yaw.minimum = 0.0
        sampling_params.yaw.maximum = 0.0
        sampling_params.radial_distance.minimum = 0.0
        sampling_params.radial_distance.maximum = 0.01

        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.2
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0

        while not self.wait_for_result:
            self.sampling_parameters.publish(sampling_params)
            self.target_pose.publish(target_pose)
            self.event_out.publish('e_start')

        self.assertEqual(
            expected_result.poses, self.result.poses,
            msg="expected: {0}\nresult: {1}".format(expected_result, self.result)
        )

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('pose_generator_test')
    rostest.rosrun(PKG, 'pose_generator_test', TestPoseGenerator)
