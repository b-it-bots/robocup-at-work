#!/usr/bin/env python
"""
This module contains functions used by the reachability_pose_selector node.

"""
#-*- encoding: utf-8 -*-

import genpy
import copy
import std_msgs.msg
import geometry_msgs.msg
import brics_actuator.msg


def pose_array_to_list(poses):
    """
    Transforms a geometry_msgs.msg.PoseArray message to
    a list of geometry_msgs.msg.PoseStamped objects.

    :param poses: An array of poses.
    :type poses: geometry_msgs.msg.PoseArray

    :return: A list of geometry_msgs.msg.PoseStamped objects.
    :rtype: list

    """
    assert (type(poses) == geometry_msgs.msg.PoseArray),\
        "'poses' must be of type 'geometry_msgs.msg.PoseArray'."

    list_of_poses = [
        geometry_msgs.msg.PoseStamped(poses.header, pose) for pose in poses.poses
    ]

    return list_of_poses


def add_linear_offset_to_pose(pose, linear_offset=None):
    """
    Adds a linear offset to a pose.

    :param pose: The pose to which the linear offset will be added.
    :type pose: geometry_msgs.msg.PoseStamped

    :param linear_offset: A linear offset for the X, Y and Z axis.
    :type linear_offset: list or None

    :return: The pose with the added linear offset.
    :rtype: geometry_msgs.msg.PoseStamped

    """
    pose_out = copy.deepcopy(pose)
    if linear_offset is None:
        return pose_out

    assert (type(linear_offset) == list and len(linear_offset) == 3),\
        "'linear_offset' must be a list of three elements (e.g. [1, 2, 3])."

    pose_out.pose.position.x = pose.pose.position.x + linear_offset[0]
    pose_out.pose.position.y = pose.pose.position.y + linear_offset[1]
    pose_out.pose.position.z = pose.pose.position.z + linear_offset[2]

    return pose_out


def list_to_brics_joints(joint_values, joint_names, time_stamp=None, unit=None):
    """
    Transforms a list of joint position values, representing a joint configuration,
    to a brics_actuator.msg.JointPosition message.

    :param joint_values: List of joint position values.
    :type joint_values: list

    :param joint_names: List of joint names, matching the order of joint_values.
    :type joint_names: list

    :param time_stamp: The time of the joint configuration.
    :type time_stamp: std_msgs.msg.Time

    :param unit: The units of the joint position values.
    :type unit: str

    :return: The transformed message.
    :rtype : brics_actuator.msg.JointPosition

    """
    assert (type(joint_values) == list and type(joint_names) == list),\
        "''joint_values' and 'joint_names' must be lists."
    assert (len(joint_values) == len(joint_names)),\
        "''joint_values' and 'joint_names' must have the same dimension."

    configuration_out = brics_actuator.msg.JointPositions()
    for _ in joint_names:
        configuration_out.positions.append(brics_actuator.msg.JointValue())

    for joint, position, name in zip(
            configuration_out.positions, joint_values, joint_names
    ):
        joint.value = position
        joint.joint_uri = name
        if unit is not None:
            joint.unit = unit
        if time_stamp is not None:
            assert (
                (type(time_stamp) == std_msgs.msg.Time) or
                (isinstance(time_stamp, genpy.Time))
            ), "'time_stamp' must be of type 'std_msgs.msg.Time' or 'genpy.Time'."

            joint.timeStamp = time_stamp

    return configuration_out