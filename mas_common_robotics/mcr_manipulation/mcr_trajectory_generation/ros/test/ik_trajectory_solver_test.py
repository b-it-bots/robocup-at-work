#!/usr/bin/env python
"""
Integration test for the 'ik_trajectory_solver' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg

PKG = 'mcr_trajectory_generation'


class TestIkTrajectorySolver(unittest.TestCase):
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
        self.poses = rospy.Publisher('~poses', geometry_msgs.msg.PoseArray, queue_size=1)

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', trajectory_msgs.msg.JointTrajectory, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.poses.unregister()
        self.component_output.unregister()

    def test_configuration_crossover(self):
        """
        Verifies that the node returns a joint trajectory.
        Note: This is not a functionality test.

        """
        pose_header = std_msgs.msg.Header(frame_id="base_link")

        pose_1 = geometry_msgs.msg.Pose()
        pose_1.position.x = 0.48
        pose_1.position.y = 0.0
        pose_1.position.z = 0.1
        pose_1.orientation.x = 0.0
        pose_1.orientation.y = 1.0
        pose_1.orientation.z = 0.0
        pose_1.orientation.w = 0.0

        pose_2 = geometry_msgs.msg.Pose()
        pose_2.position.x = 0.49
        pose_2.position.y = 0.0
        pose_2.position.z = 0.1
        pose_2.orientation.x = 0.0
        pose_2.orientation.y = 1.0
        pose_2.orientation.z = 0.0
        pose_2.orientation.w = 0.0

        pose_3 = geometry_msgs.msg.Pose()
        pose_3.position.x = 0.5
        pose_3.position.y = 0.0
        pose_3.position.z = 0.1
        pose_3.orientation.x = 0.0
        pose_3.orientation.y = 1.0
        pose_3.orientation.z = 0.0
        pose_3.orientation.w = 0.0

        poses = geometry_msgs.msg.PoseArray(
            header=pose_header, poses=[pose_1, pose_2, pose_3]
        )

        while not self.wait_for_result:
            self.event_out.publish('e_start')
            self.poses.publish(poses)

        assert type(self.result) is trajectory_msgs.msg.JointTrajectory

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('ik_trajectory_solver_test')
    rostest.rosrun(PKG, 'ik_trajectory_solver_test', TestIkTrajectorySolver)
