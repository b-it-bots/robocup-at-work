#!/usr/bin/env python
"""
This module contains functions used by the ik_trajectory_solver node.

"""
#-*- encoding: utf-8 -*-

import copy
import genpy
import numpy
import std_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg


def limit_number_of_poses(poses, max_poses=10):
    """
    Given a number of poses and a limit (max_poses), returns only the limit
    amount of poses (or the complete list if the limit is more than the poses).
    If max_poses is more than the half number of poses, then the first
    'max_poses - 1' are returned plus the last pose.

    The returned poses are equally separated and the first and last poses remain
    as first and last in the returned poses.


    :param poses: List of poses.
    :type poses: geometry_msgs.msg.PoseArray

    :param max_poses: The maximum number of poses to return. It should be greater
        or equal to 2.
    :type max_poses: int

    :return: A list of 'n' poses, where 'n' is equal to 'max_poses'.
    :rtype: geometry_msgs.msg.PoseArray

    """
    assert type(poses) == geometry_msgs.msg.PoseArray,\
        "''poses' must be of type 'geometry_msgs.msg.PoseArray'."
    assert max_poses > 1, "'max_poses' must be greater or equal than 2."

    poses_out = copy.deepcopy(poses)
    if len(poses_out.poses) <= max_poses:
        return poses_out

    chunk_size = int(len(poses.poses) / float(max_poses))
    temp_poses = [
        poses.poses[i: i + chunk_size] for i in xrange(0, len(poses.poses), chunk_size)
    ]
    poses_out.poses = [
        temp_pose[chunk_size / 2] for ii, temp_pose in enumerate(temp_poses)
        if ii < max_poses
    ]

    # ensure the first pose matches the last one from 'poses'
    poses_out.poses[0] = poses.poses[0]

    # ensure the right amount of poses are returned
    while len(poses_out.poses) > max_poses:
        poses_out.poses.pop()

    # ensure the last pose matches the last one from 'poses'
    if len(poses_out.poses) == max_poses:
        poses_out.poses[-1] = poses.poses[-1]
    elif len(poses_out.poses) < max_poses:
        poses_out.poses.append(poses.poses[-1])

    return poses_out


def list_to_joint_trajectory(
        configurations, joint_names, time_stamp=None, frame_id=None
):
    """
    Transforms a list of joint configurations to a
    trajectory_msgs.msg.JointTrajectory message.

    :param configurations: List of joint configurations.
    :type configurations: list

    :param joint_names: List of joint names, matching the order of joint_configurations.
    :type joint_names: list

    :param time_stamp: The time of the joint configuration.
    :type time_stamp: std_msgs.msg.Time

    :param frame_id: The reference frame of the trajectory.
    :type frame_id: str

    :return: The transformed message.
    :rtype: trajectory_msgs.msg.JointTrajectory

    """
    assert (type(configurations) == list and type(joint_names) == list),\
        "''configurations' and 'joint_names' must be lists."

    assert numpy.array_equal(
        [type(conf) for conf in configurations], [list for _ in configurations]
    ), "All elements in 'configurations' must be lists."

    dimensions = [len(configuration) for configuration in configurations]
    assert numpy.array_equal(dimensions, [dimensions[0] for _ in configurations]),\
        "'configurations' must be of the same dimension."

    assert (dimensions[0] == len(joint_names)),\
        "'configurations' must have the same dimension as 'joint_names'."

    configuration_out = trajectory_msgs.msg.JointTrajectory()
    configuration_out.header.frame_id = frame_id
    configuration_out.joint_names = joint_names
    configuration_out.points = [
        trajectory_msgs.msg.JointTrajectoryPoint(positions=configuration)
        for configuration in configurations
    ]

    if time_stamp is not None:
        assert (
            (type(time_stamp) == std_msgs.msg.Time) or
            (isinstance(time_stamp, genpy.Time))
        ), "'time_stamp' must be of type 'std_msgs.msg.Time' or 'genpy.Time'."

        configuration_out.header.stamp = time_stamp

    return configuration_out
