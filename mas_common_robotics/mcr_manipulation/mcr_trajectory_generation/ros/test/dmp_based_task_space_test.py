#!/usr/bin/env python
"""
Integration test for the 'dmp_based_task_space' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg

PKG = 'mcr_trajectory_generation'


class TestDmpBasedTaskSpace(unittest.TestCase):
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
        Verifies that the node returns a joint trajectory.
        Note: This is not a functionality test.

        """
        start_pose = geometry_msgs.msg.PoseStamped()
        goal_pose = geometry_msgs.msg.PoseStamped()
        start_pose.header.frame_id = "base_link"
        goal_pose.header.frame_id = "base_link"

        start_pose.pose.position.x = 1.0
        start_pose.pose.position.y = 2.0
        start_pose.pose.position.z = 3.0
        start_pose.pose.orientation.x = 0.0
        start_pose.pose.orientation.y = 0.0
        start_pose.pose.orientation.z = 1.0
        start_pose.pose.orientation.w = 0.0

        goal_pose.pose.position.x = 1.0
        start_pose.pose.position.y = 2.0
        goal_pose.pose.position.z = 3.0
        # 180 degrees rotation around the X axis
        goal_pose.pose.orientation.x = 1.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        while not self.wait_for_result:
            self.event_out.publish('e_start')
            self.start_pose.publish(start_pose)
            self.goal_pose.publish(goal_pose)

        assert type(self.result) is geometry_msgs.msg.PoseArray

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('dmp_based_task_space_test')
    rostest.rosrun(PKG, 'dmp_based_task_space_test', TestDmpBasedTaskSpace)
