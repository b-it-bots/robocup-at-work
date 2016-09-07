#!/usr/bin/env python
"""
Integration test for the 'pose_shifter_pipeline' node.

"""

import unittest
import rospy
import rostest
import std_msgs.msg
import geometry_msgs.msg

PKG = 'mcr_common_converters'


class TestPoseShifterPipeline(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, latch=True)
        self.pose_in_pub = rospy.Publisher('~pose_in', geometry_msgs.msg.PoseStamped, latch=True)

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', geometry_msgs.msg.PoseStamped, self.pose_out_cb
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.component_output.unregister()

    def test_pose_shifter_pipeline(self):
        """
        Verifies that the node returns a pose.
        Note: This is not a functionality test.

        """
        pose_in = geometry_msgs.msg.PoseStamped()
        self.pose_in_pub.publish(pose_in)
        while not self.wait_for_result:
            self.event_out.publish('e_start')

        assert type(self.result) is geometry_msgs.msg.PoseStamped

    def pose_out_cb(self, msg):
        """
        Obtains the result of the component.

        """
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('pregrasp_planner_pipeline_test')
    rostest.rosrun(PKG, 'pregrasp_planner_pipeline_test', TestPoseShifterPipeline)
