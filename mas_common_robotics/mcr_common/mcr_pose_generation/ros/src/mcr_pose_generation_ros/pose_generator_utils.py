#!/usr/bin/env python
"""
This module contains functions used by the pose_generator node.

"""
#-*- encoding: utf-8 -*-

import numpy
import tf
import rospy
import geometry_msgs.msg


def generate_samples(min_value, max_value, step, max_samples=50):
    """
    Generates an evenly spaced list of samples between the minimum values (min_value)
    and the maximum value (max_value). The minimum and maximum values are both
    inclusive. If the number of samples exceeds the max_samples argument, then only
    max_samples are returned.

    Note: The step might be changed slightly in order to create
          an evenly spaced list.

    :param min_value: The minimum value of the samples.
    :type min_value: float

    :param max_value: The maximum value of the samples.
    :type max_value: float

    :param step: The (approximate) step between samples.
    :type step: float

    :param step: The maximum number of samples allowed.
    :type step: int

    :return: A list of samples.
    :rtype: float[]

    """
    number_of_samples = round((max_value - min_value) / float(step))
    number_of_samples = min([number_of_samples, max_samples - 1])

    return numpy.linspace(min_value, max_value, int(number_of_samples + 1))


def matrix_to_pose(matrix):
    """
    Converts a transformation matrix into a pose, in which the
    orientation is specified by a quaternion.

    :param matrix: The 4x4 transformation matrix.
    :type matrix: numpy.array

    :return: The pose interpretable by ROS.
    :rtype: geometry_msgs.msg.Pose()

    """
    pose = geometry_msgs.msg.Pose()

    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]

    quaternion = tf.transformations.quaternion_from_matrix(matrix)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose
