#!/usr/bin/env python
"""
This module contains functions used by the linear_interpolator node.

"""
#-*- encoding: utf-8 -*-

import numpy
import collections
import geometry_msgs.msg


def interpolate_point(point_1, point_2, distance):
    """
    Returns a n-dimensional point that lies between two n-dimensional points
    at a specified 'distance' from 'point_1'.

    :param point_1: An n-dimensional point.
    :type point_1: list

    :param point_2: An n-dimensional point.
    :type point_2: list

    :param distance: The distance from the specified 'point' where the new point
    should be created.
    :type distance: float

    :return: The created n-dimensional point.
    :rtype: list

    """
    assert type(distance) == float,\
        "'distance' must be of type float, not {0}".format(type(distance).__name__)
    assert isinstance(point_1, collections.Sequence),\
        "'point_1' must be of type list, not {0}".format(type(point_1).__name__)
    assert isinstance(point_2, collections.Sequence),\
        "'point_2' must be of type list, not {0}".format(type(point_2).__name__)
    assert len(point_1) == len(point_2),\
        "'point_1' and 'point_2' must be of the same size."

    p1 = numpy.array(point_1)
    p2 = numpy.array(point_2)

    vector = p2 - p1

    step = (distance / float(numpy.linalg.norm(vector)))
    new_point = p1 + (step * vector)

    return new_point.tolist()


def interpolate_poses(start_pose, goal_pose, max_poses):
    """
    Creates an amount 'max_points' of poses, equally spaced, between the start pose
    and goal pose.
    Assumption: The generated poses will have the same orientation as the start pose.

    :param start_pose: The first pose of the path.
    :type start_pose: geometry_msgs.msg.PoseStamped

    :param goal_pose: The last pose of the path.
    :type goal_pose: geometry_msgs.msg.PoseStamped

    :param max_poses: Number of point to generate between the start and goal pose
    (excluding those two).
    :type max_poses: int

    :return: The path of poses connecting the start and goal poses.
    :rtype: list

    """
    assert (type(start_pose) == geometry_msgs.msg.PoseStamped),\
        "'start_pose' must be of type 'PoseStamped'."
    assert (type(goal_pose) == geometry_msgs.msg.PoseStamped),\
        "'goal_pose' must be of type 'PoseStamped'."
    assert type(max_poses) == int,\
        "'max_poses' must be of type max_poses, not {0}".format(type(max_poses).__name__)

    point_1 = pose_stamped_to_list(start_pose)[0:3]
    point_2 = pose_stamped_to_list(goal_pose)[0:3]
    vector = numpy.linalg.norm(numpy.array(point_2) - numpy.array(point_1))

    # Add an extra 2 for the start and end poses, these will be removed afterwards.
    poses_to_calculate = max_poses + 2
    distances = numpy.linspace(0, vector, num=poses_to_calculate, endpoint=True)

    interpolated_points = [
        interpolate_point(point_1, point_2, float(d)) for d in distances
    ]

    # remove the start and end poses
    interpolated_points.pop(0)
    interpolated_points.pop()

    interpolated_poses = [list_to_pose(pp) for pp in interpolated_points]
    for pose in interpolated_poses:
        pose.orientation = start_pose.pose.orientation

    return interpolated_poses


def pose_stamped_to_list(pose):
    """
    Converts a PoseStamped message into a seven-dimensional list.

    :param pose: The pose to convert into a list.
    :type pose: geometry_msgs.msg.PoseStamped

    :return: A seven-dimensional list representing the position
        and orientation values of a pose.
    :rtype: list

    """
    assert (type(pose) == geometry_msgs.msg.PoseStamped),\
        "'pose' must be of type 'PoseStamped'."
    return [
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        pose.pose.orientation.x, pose.pose.orientation.y,
        pose.pose.orientation.z, pose.pose.orientation.w
    ]


def list_to_pose(point):
    """
    Converts a three-dimensional point, specifying the position in X, Y and Z; into
    a Pose message.

    :param point: The three-dimensional point pose to convert into a Pose.
    :type point: list

    :return: A pose.
    :rtype: geometry_msgs.msg.Pose

    """
    assert isinstance(point, collections.Sequence),\
        "'point' must be of type list, not {0}".format(type(point).__name__)
    assert len(point) == 3,\
        "'point' must be three-dimensional."
    pose = geometry_msgs.msg.Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.position.z = point[2]

    return pose
