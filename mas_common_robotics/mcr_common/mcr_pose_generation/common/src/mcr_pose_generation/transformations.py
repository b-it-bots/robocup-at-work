#!/usr/bin/env python
"""
This module contains utility functions to calculate transformation matrices.

"""
#-*- encoding: utf-8 -*-

import math
import numpy


def generate_grasp_matrix(
        object_pose, gripper_config_matrix, height_offset,
        zenith, azimuth, wrist_roll, radial
):
    """
    Assumption: The object's reference frame is assumed to be aligned
    with the robot's base frame, but offset by a translation.

    Generates a transformation matrix to align the end effector according
    to the polar coordinates: radial, azimuthal and zenith [1]; plus a height
    offset and a wrist roll.

    :param object_pose: The 4x4 transformation matrix describing the
    object's pose. The following assumptions are made:
        - X axis is along the object's longest axis.
        - If the object is standing (e.g. vertical orientation):
            - Y axis is along the object's second longest axis.
        - If the objects is laying (e.g. horizontal orientation):
            - Z axis points upwards.
    :type object_pose: numpy.array

    :param gripper_config_matrix: The 4x4 transformation matrix describing the
    pose of the gripper with respect to the robot's base frame. If an identity
    matrix is provided then:
        - The approach direction is aligned with the Z axis of the robot's frame.
        - The major axis along is aligned with the Y axis of the robot's frame.
        - The last axis is aligned with the X axis of the robot's frame.
    This matrix should also contain the desired position of the grasp.
    :type gripper_config_matrix: numpy.array

    :param height_offset: The height offset, with respect to the object, where
    the orbit is centered.
    :type height_offset: float

    :param zenith: The zenith angle (polar angle) in spherical coordinates.
    This angle is input to a rotation matrix about the Y axis.
    :type zenith: float

    :param azimuth: The azimuthal angle in spherical coordinates.
    This angle is input to a rotation matrix about the X axis.
    :type azimuth: float

    :param wrist_roll: The rotation angle about the Z axis of the end effector.
    :type wrist_roll: float

    :param radial: The distance between the object and the end effector.
    :type radial: float

    :return: The 4x4 transformation matrix representing the end effector's pose.
    :rtype: numpy.array

    [1] http://mathworld.wolfram.com/SphericalCoordinates.html

    """
    # Add distance between object and end effector
    radial_transform = numpy.array([
        [1.0, 0.0, 0.0,     0.0],
        [0.0, 1.0, 0.0,     0.0],
        [0.0, 0.0, 1.0, -radial],
        [0.0, 0.0, 0.0,     1.0]
    ])

    # Add a height offset to the object
    height_transform = numpy.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0,           0.0],
        [0.0, 0.0, 1.0, height_offset],
        [0.0, 0.0, 0.0,           1.0]
    ])

    # Rotation around the Y axis of end effector
    zenith_transform = numpy.array([
        [math.cos(zenith),  0.0, math.sin(zenith), 0.0],
        [0.0,               1.0,              0.0, 0.0],
        [-math.sin(zenith), 0.0, math.cos(zenith), 0.0],
        [0.0,               0.0,              0.0, 1.0]
    ])

    # Rotation around the X axis of the end effector
    azimuth_transform = numpy.array([
        [1.0,               0.0,                0.0, 0.0],
        [0.0, math.cos(azimuth), -math.sin(azimuth), 0.0],
        [0.0, math.sin(azimuth),  math.cos(azimuth), 0.0],
        [0.0,               0.0,                0.0, 1.0]
    ])

    # Rotation around the Z axis of the end effector
    wrist_orientation = numpy.array([
        [math.cos(wrist_roll), -math.sin(wrist_roll), 0.0, 0.0],
        [math.sin(wrist_roll),  math.cos(wrist_roll), 0.0, 0.0],
        [0.0,                                    0.0, 1.0, 0.0],
        [0.0,                                    0.0, 0.0, 1.0]
    ])

    # Transform equations:
    # The second last transformation should be to orient the gripper
    # as to match the 'gripper_config_matrix' and the last transformation
    # should be the distance between the end-effector and the object.
    t_1 = numpy.dot(height_transform, object_pose)
    t_2 = numpy.dot(t_1, zenith_transform)
    t_3 = numpy.dot(t_2, azimuth_transform)
    t_4 = numpy.dot(t_3, wrist_orientation)
    t_5 = numpy.dot(t_4, gripper_config_matrix)
    t_6 = numpy.dot(t_5, radial_transform)

    return t_6
