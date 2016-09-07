#!/usr/bin/env python
"""
Integration test for the 'trajectory_generator' node.

"""

import rospy
import numpy.testing as testing
import unittest
import rostest
import mcr_trajectory_generation_ros.ik_trajectory_solver_utils as utils
import std_msgs.msg
import trajectory_msgs.msg

PKG = 'mcr_trajectory_generation'


class TestTrajectoryGenerator(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, latch=True)
        self.trajectory = rospy.Publisher(
            '~trajectory', trajectory_msgs.msg.JointTrajectory, queue_size=1
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', trajectory_msgs.msg.JointTrajectory, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.trajectory.unregister()
        self.component_output.unregister()

    def test_trajectory_generation(self):
        """
        Verifies that the node returns a correct trajectory.

        """
        zero_vector = [0, 0, 0]
        configurations = [
            [1.0, 2.0, 3.0], [1.1, 2.2, 2.7], [0.9, 1.2, 3.2], [0.7, 1.3, 3.5]
        ]
        joint_names = ['a', 'b', 'c']
        trajectory = utils.list_to_joint_trajectory(configurations, joint_names)

        expected_velocities_1 = [0.1, 0.2, -0.3]
        expected_velocities_2 = [-0.2, -1.0, 0.5]
        expected_velocities_3 = [-0.2, 0.1, 0.3]
        expected_accelerations_1 = [-0.3, -1.2, 0.8]
        expected_accelerations_2 = [0.0, 1.1, -0.2]
        expected_accelerations_3 = [0.2, -0.1, -0.3]

        while not self.wait_for_result:
            self.event_out.publish('e_start')
            self.trajectory.publish(trajectory)

        # Last point should have zero velocity and acceleration
        testing.assert_array_equal(self.result.points[-1].velocities, zero_vector)
        testing.assert_array_equal(self.result.points[-1].accelerations, zero_vector)

        # Test velocity and acceleration
        testing.assert_array_almost_equal(
            self.result.points[0].velocities, expected_velocities_1
        )
        testing.assert_array_almost_equal(
            self.result.points[0].accelerations, expected_accelerations_1
        )
        testing.assert_array_almost_equal(
            self.result.points[1].velocities, expected_velocities_2
        )
        testing.assert_array_almost_equal(
            self.result.points[1].accelerations, expected_accelerations_2
        )
        testing.assert_array_almost_equal(
            self.result.points[2].velocities, expected_velocities_3
        )
        testing.assert_array_almost_equal(
            self.result.points[2].accelerations, expected_accelerations_3
        )

        # Test time from start
        self.assertEqual(self.result.points[0].time_from_start, rospy.Duration(0))
        self.assertAlmostEqual(self.result.points[1].time_from_start.nsecs * 1e-9, 0.3)
        self.assertAlmostEqual(self.result.points[2].time_from_start.secs, 1.0)
        self.assertAlmostEqual(self.result.points[3].time_from_start.secs, 1.0)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('trajectory_generator_test')
    rostest.rosrun(PKG, 'trajectory_generator_test', TestTrajectoryGenerator)
