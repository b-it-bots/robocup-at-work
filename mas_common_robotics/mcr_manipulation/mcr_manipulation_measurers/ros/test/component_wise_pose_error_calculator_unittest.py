#!/usr/bin/env python
"""
Test unit for the functions/methods used in component_wise_pose_error_calculator.py module.

"""

PKG = 'mcr_manipulation_measurers'

import math
import unittest
import rosunit
import geometry_msgs.msg
import mcr_manipulation_msgs.msg
import mcr_manipulation_measurers_ros.component_wise_pose_error_calculator \
    as component_wise_pose_error_calculator


class UnitTestComponentWisePoseErrorCalculator(unittest.TestCase):
    """
    Tests methods used in the component_wise_pose_error_calculator.py module.

    """
    def test_linear_distances(self):
        """
        Tests that the component returns the correct linear distances
        between two PoseStamped objects.

        """
        current_pose_1 = geometry_msgs.msg.PoseStamped()
        current_pose_2 = geometry_msgs.msg.PoseStamped()
        current_pose_3 = geometry_msgs.msg.PoseStamped()

        target_pose = geometry_msgs.msg.PoseStamped()

        ## assign test values ##
        current_pose_1.pose.position.x = 1.5
        current_pose_1.pose.position.y = 0.0
        current_pose_1.pose.position.z = 0.0
        current_pose_1.pose.orientation.x = 0.0
        current_pose_1.pose.orientation.y = 0.0
        current_pose_1.pose.orientation.z = 0.0
        current_pose_1.pose.orientation.w = 1.0

        current_pose_2.pose.position.x = 2.0
        current_pose_2.pose.position.y = 0.0
        current_pose_2.pose.position.z = 3.7
        current_pose_2.pose.orientation.x = 0.0
        current_pose_2.pose.orientation.y = 0.0
        current_pose_2.pose.orientation.z = 0.0
        current_pose_2.pose.orientation.w = 1.0

        current_pose_3.pose.position.x = 8.5
        current_pose_3.pose.position.y = 4.1
        current_pose_3.pose.position.z = 3.9
        current_pose_3.pose.orientation.x = 0.0
        current_pose_3.pose.orientation.y = 0.0
        current_pose_3.pose.orientation.z = 0.0
        current_pose_3.pose.orientation.w = 1.0

        target_pose.pose.position.x = 3.0
        target_pose.pose.position.y = 4.0
        target_pose.pose.position.z = 5.0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0

        expected_result_1 = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        expected_result_1.linear.x = 1.5
        expected_result_1.linear.y = 4.0
        expected_result_1.linear.z = 5.0

        expected_result_2 = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        expected_result_2.linear.x = 1.0
        expected_result_2.linear.y = 4.0
        expected_result_2.linear.z = 1.3

        expected_result_3 = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        expected_result_3.linear.x = -5.5
        expected_result_3.linear.y = -0.1
        expected_result_3.linear.z = 1.1
        ########################

        result_1 = component_wise_pose_error_calculator.\
            calculate_component_wise_pose_error(current_pose_1, target_pose)
        result_2 = component_wise_pose_error_calculator.\
            calculate_component_wise_pose_error(current_pose_2, target_pose)
        result_3 = component_wise_pose_error_calculator.\
            calculate_component_wise_pose_error(current_pose_3, target_pose)

        self.assertAlmostEqual(expected_result_1.linear.x, result_1.linear.x)
        self.assertAlmostEqual(expected_result_1.linear.y, result_1.linear.y)
        self.assertAlmostEqual(expected_result_1.linear.z, result_1.linear.z)

        self.assertAlmostEqual(expected_result_2.linear.x, result_2.linear.x)
        self.assertAlmostEqual(expected_result_2.linear.y, result_2.linear.y)
        self.assertAlmostEqual(expected_result_2.linear.z, result_2.linear.z)

        self.assertAlmostEqual(expected_result_3.linear.x, result_3.linear.x)
        self.assertAlmostEqual(expected_result_3.linear.y, result_3.linear.y)
        self.assertAlmostEqual(expected_result_3.linear.z, result_3.linear.z)

    def test_linear_distances_with_offsets(self):
        """
        Tests that the component returns the correct linear distances
        between two PoseStamped objects including offsets.

        """
        offset_1 = (0.0, 0.0, 0.0)
        offset_2 = (2.0, 2.0, 2.0)
        offset_3 = (-2.0, 5.0, 0.5)

        current_pose_1 = geometry_msgs.msg.PoseStamped()
        current_pose_2 = geometry_msgs.msg.PoseStamped()
        current_pose_3 = geometry_msgs.msg.PoseStamped()

        target_pose = geometry_msgs.msg.PoseStamped()

        ## assign test values ##
        current_pose_1.pose.position.x = 1.5
        current_pose_1.pose.position.y = 0.0
        current_pose_1.pose.position.z = 0.0

        current_pose_2.pose.position.x = 2.0
        current_pose_2.pose.position.y = 0.0
        current_pose_2.pose.position.z = 3.7

        current_pose_3.pose.position.x = 8.5
        current_pose_3.pose.position.y = 4.1
        current_pose_3.pose.position.z = 3.9

        target_pose.pose.position.x = 3.0
        target_pose.pose.position.y = 4.0
        target_pose.pose.position.z = 5.0

        expected_result_1 = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        expected_result_1.linear.x = 1.5 + offset_1[0]
        expected_result_1.linear.y = 4.0 + offset_1[1]
        expected_result_1.linear.z = 5.0 + offset_1[2]

        expected_result_2 = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        expected_result_2.linear.x = 1.0 + offset_2[0]
        expected_result_2.linear.y = 4.0 + offset_2[1]
        expected_result_2.linear.z = 1.3 + offset_2[2]

        expected_result_3 = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        expected_result_3.linear.x = -5.5 + offset_3[0]
        expected_result_3.linear.y = -0.1 + offset_3[1]
        expected_result_3.linear.z = 1.1 + offset_3[2]
        ########################

        result_1 = component_wise_pose_error_calculator.\
            calculate_component_wise_pose_error(current_pose_1, target_pose, offset_1)
        result_2 = component_wise_pose_error_calculator.\
            calculate_component_wise_pose_error(current_pose_2, target_pose, offset_2)
        result_3 = component_wise_pose_error_calculator.\
            calculate_component_wise_pose_error(current_pose_3, target_pose, offset_3)

        self.assertAlmostEqual(expected_result_1.linear.x, result_1.linear.x)
        self.assertAlmostEqual(expected_result_1.linear.y, result_1.linear.y)
        self.assertAlmostEqual(expected_result_1.linear.z, result_1.linear.z)

        self.assertAlmostEqual(expected_result_2.linear.x, result_2.linear.x)
        self.assertAlmostEqual(expected_result_2.linear.y, result_2.linear.y)
        self.assertAlmostEqual(expected_result_2.linear.z, result_2.linear.z)

        self.assertAlmostEqual(expected_result_3.linear.x, result_3.linear.x)
        self.assertAlmostEqual(expected_result_3.linear.y, result_3.linear.y)
        self.assertAlmostEqual(expected_result_3.linear.z, result_3.linear.z)

    def test_angular_distances(self):
        """
        Tests that the component returns the correct angular distances
        between two PoseStamped objects.

        """
        current_pose_1 = geometry_msgs.msg.PoseStamped()
        current_pose_2 = geometry_msgs.msg.PoseStamped()
        current_pose_3 = geometry_msgs.msg.PoseStamped()

        target_pose = geometry_msgs.msg.PoseStamped()

        current_pose_1.pose.position.x = 1.5
        current_pose_1.pose.position.y = 0.0
        current_pose_1.pose.position.z = 0.0

        current_pose_2.pose.position.x = 2.0
        current_pose_2.pose.position.y = 0.0
        current_pose_2.pose.position.z = 3.7

        current_pose_3.pose.position.x = 8.5
        current_pose_3.pose.position.y = 4.1
        current_pose_3.pose.position.z = 3.9

        target_pose.pose.position.x = 3.0
        target_pose.pose.position.y = 4.0
        target_pose.pose.position.z = 5.0

        ## assign test values ##
        # 90 degree 'yaw' rotation
        current_pose_1.pose.orientation.w = 0.7071
        current_pose_1.pose.orientation.x = 0.0
        current_pose_1.pose.orientation.y = 0.0
        current_pose_1.pose.orientation.z = 0.7071

        # 45 degree 'pitch' rotation
        current_pose_2.pose.orientation.w = 0.9239
        current_pose_2.pose.orientation.x = 0.0
        current_pose_2.pose.orientation.y = 0.3827
        current_pose_2.pose.orientation.z = 0.0

        # 30 degree 'roll' rotation
        current_pose_3.pose.orientation.w = 0.9659
        current_pose_3.pose.orientation.x = 0.2588
        current_pose_3.pose.orientation.y = 0.0
        current_pose_3.pose.orientation.z = 0.0

        target_pose.pose.orientation.w = 1.0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0

        expected_result_1 = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        expected_result_1.angular.x = 0.0
        expected_result_1.angular.y = 0.0
        expected_result_1.angular.z = -math.pi/2

        expected_result_2 = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        expected_result_2.angular.x = 0.0
        expected_result_2.angular.y = -math.pi/4
        expected_result_2.angular.z = 0.0

        expected_result_3 = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
        expected_result_3.angular.x = -math.pi/6
        expected_result_3.angular.y = 0.0
        expected_result_3.angular.z = 0.0
        ########################

        result_1 = component_wise_pose_error_calculator.\
            calculate_component_wise_pose_error(current_pose_1, target_pose)
        result_2 = component_wise_pose_error_calculator.\
            calculate_component_wise_pose_error(current_pose_2, target_pose)
        result_3 = component_wise_pose_error_calculator.\
            calculate_component_wise_pose_error(current_pose_3, target_pose)

        self.assertAlmostEqual(
            expected_result_1.angular.x, result_1.angular.x, places=4
        )
        self.assertAlmostEqual(
            expected_result_1.angular.y, result_1.angular.y, places=4
        )
        self.assertAlmostEqual(
            expected_result_1.angular.z, result_1.angular.z, places=4
        )

        self.assertAlmostEqual(
            expected_result_2.angular.x, result_2.angular.x, places=4
        )
        self.assertAlmostEqual(
            expected_result_2.angular.y, result_2.angular.y, places=4
        )
        self.assertAlmostEqual(
            expected_result_2.angular.z, result_2.angular.z, places=4
        )

        self.assertAlmostEqual(
            expected_result_3.angular.x, result_3.angular.x, places=4
        )
        self.assertAlmostEqual(
            expected_result_3.angular.y, result_3.angular.y, places=4
        )
        self.assertAlmostEqual(
            expected_result_3.angular.z, result_3.angular.z, places=4
        )


if __name__ == '__main__':
    rosunit.unitrun(
        PKG, 'test_component_wise_pose_error_calculator',
        UnitTestComponentWisePoseErrorCalculator
    )
