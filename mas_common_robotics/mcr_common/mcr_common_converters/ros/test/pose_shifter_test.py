#!/usr/bin/env python
"""
Integration test for the 'pose_shifter' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import dynamic_reconfigure.client

PKG = 'mcr_common_converters'


class TestPoseShifter(unittest.TestCase):
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
        self.pose_in_pub = rospy.Publisher(
            '~pose_in', geometry_msgs.msg.PoseStamped, latch=True
        )
        
        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', geometry_msgs.msg.PoseStamped, self.result_callback
        )
        self.dyn_client = dynamic_reconfigure.client.Client("pose_shifter", timeout=15)




    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.pose_in_pub.unregister()
        self.event_out.unregister()
        self.component_output.unregister()

    def test_pose_shifter(self):
        """
        Verifies that the node returns a shifted pose according to the
        specified linear offset.

        """
        self.dyn_client.update_configuration({"linear_offset_x":0.1, "linear_offset_y":0.0, "linear_offset_z":0.05})
        pose_in = geometry_msgs.msg.PoseStamped()
        expected = geometry_msgs.msg.PoseStamped()
        pose_in.header.frame_id = "base_link"
        expected.header.frame_id = "base_link"

        pose_in.pose.position.x = 1.0
        pose_in.pose.position.y = 2.0
        pose_in.pose.position.z = 3.0
        pose_in.pose.orientation.x = 0.0
        pose_in.pose.orientation.y = 0.0
        pose_in.pose.orientation.z = 0.0
        pose_in.pose.orientation.w = 1.0

        # shift of 10 cm in X and 5 cm in Z
        expected.pose.position.x = 1.1
        expected.pose.position.y = 2.0
        expected.pose.position.z = 3.05
        expected.pose.orientation.x = 0.0
        expected.pose.orientation.y = 0.0
        expected.pose.orientation.z = 0.0
        expected.pose.orientation.w = 1.0

        self.pose_in_pub.publish(pose_in)

        while not self.wait_for_result:
            self.event_out.publish('e_start')

        self.assertEqual(self.result.header.frame_id, expected.header.frame_id)
        self.assertEqual(self.result.pose, expected.pose)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('pose_shifter_test')
    rostest.rosrun(PKG, 'pose_shifter_test', TestPoseShifter)
