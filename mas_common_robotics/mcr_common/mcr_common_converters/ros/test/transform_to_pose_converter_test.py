#!/usr/bin/env python
"""
Integration test for the 'transform_to_pose_converter' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import tf

PKG = 'mcr_common_converters'


class TestTransformToPoseConverter(unittest.TestCase):
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

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', geometry_msgs.msg.PoseStamped, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()

    def test_cartesian_distance_controller(self):
        """
        Verifies that the node returns correctly a Pose Stamped,
        based on a target frame and the reference frame.

        """
        expected_result = geometry_msgs.msg.PoseStamped()
        expected_result.pose.position.x = 1.0
        expected_result.pose.position.y = 0.0
        expected_result.pose.position.z = 0.0
        expected_result.pose.orientation.x = 0.0
        expected_result.pose.orientation.y = 0.0
        expected_result.pose.orientation.z = 0.0
        expected_result.pose.orientation.w = 1.0

        reference_frame = 'base_link'
        target_frame = 'arm_link_5'

        translation = (1.0, 0.0, 0.0)
        rotation = (0.0, 0.0, 0.0, 1.0)

        broadcaster = tf.TransformBroadcaster()

        while not self.wait_for_result:
            broadcaster.sendTransform(
                translation, rotation, rospy.Time.now(),
                target_frame, reference_frame
            )
            self.event_out.publish('e_start')

        self.assertAlmostEqual(
            self.result.pose.position.x, expected_result.pose.position.x
        )
        self.assertAlmostEqual(
            self.result.pose.orientation.w, expected_result.pose.orientation.w
        )

        # reset component
        self.event_out.publish('e_stop')
        rospy.sleep(0.5)
        self.wait_for_result = False

        # update tf
        translation = (2.0, 0.0, 0.0)
        expected_result.pose.position.x = 2.0

        while not self.wait_for_result:
            broadcaster.sendTransform(
                translation, rotation, rospy.Time.now(),
                target_frame, reference_frame
            )
            self.event_out.publish('e_start')

        self.assertAlmostEqual(
            self.result.pose.position.x, expected_result.pose.position.x
        )

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('transform_to_pose_converter_test')
    rostest.rosrun(PKG, 'transform_to_pose_converter_test', TestTransformToPoseConverter)
