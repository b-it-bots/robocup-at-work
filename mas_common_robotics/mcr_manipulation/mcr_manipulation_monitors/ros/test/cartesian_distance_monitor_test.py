#!/usr/bin/env python
"""
Integration test for the 'cartesian_distance_monitor' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import tf

PKG = 'mcr_manipulation_monitors'


class TestCartesianDistanceMonitor(unittest.TestCase):
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
        self.pose_1 = rospy.Publisher(
            '~pose_1', geometry_msgs.msg.PoseStamped
        )
        self.pose_2 = rospy.Publisher(
            '~pose_2', geometry_msgs.msg.PoseStamped
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', std_msgs.msg.String, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.pose_2.unregister()
        self.pose_1.unregister()

    def test_cartesian_distance_monitor(self):
        """
        Verifies that the node correctly detects that the distance
        between two poses is within the specified tolerance.

        """
        expected_result = std_msgs.msg.String()
        expected_result.data = 'e_done'

        current = geometry_msgs.msg.PoseStamped()
        current.header.frame_id = 'base_link'
        current.pose.position.x = 0.0
        current.pose.position.y = 0.0
        current.pose.position.z = 0.0
        current.pose.orientation.x = 0.0
        current.pose.orientation.y = 0.0
        current.pose.orientation.z = 0.0
        current.pose.orientation.w = 1.0

        target = geometry_msgs.msg.PoseStamped()
        target.header.frame_id = 'arm_link_5'
        target.pose.position.x = 0.0
        target.pose.position.y = 0.0
        target.pose.position.z = 0.0
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 1.0

        translation = (0.251, -0.016, 0.411)
        rotation = (0.0, 0.0, 0.0, 1.0)
        reference_frame = 'base_link'
        target_frame = 'arm_link_5'

        broadcaster = tf.TransformBroadcaster()

        while not self.wait_for_result:
            broadcaster.sendTransform(
                translation, rotation, rospy.Time.now(), target_frame, reference_frame
            )
            self.pose_1.publish(target)
            self.pose_2.publish(current)
            self.event_out.publish('e_start')

        self.assertEqual(self.result.data, expected_result.data)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('cartesian_distance_monitor_test')
    rostest.rosrun(PKG, 'cartesian_distance_monitor_test', TestCartesianDistanceMonitor)
