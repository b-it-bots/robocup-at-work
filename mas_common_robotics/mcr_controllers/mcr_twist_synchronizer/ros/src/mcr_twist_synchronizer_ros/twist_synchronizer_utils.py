#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This module contains functions used by the twist_synchronizer node.

"""


def calculate_max_time(error, velocity, angular_synchronization=False, zero=0.001):
    """
    Calculates the maximum time, between all the velocities, required to reach
    the goal. By default, it only synchronizes linear velocities.
    If angular_synchronization is True, then it also synchronizes for angular
    velocities.

    :param error: The distance error tuple in the X, Y and Z axis.
    :type error: list

    :param velocity: Velocities in the X, Y and Z axis.
    :type velocity: list

    :param zero: Value to prevent division by near-zero values.
    :type zero: float

    :return: The maximum time required to reach the goal.
    :rtype: float

    """
    if angular_synchronization:
        assert len(error) == len(velocity) == 6
    else:
        assert len(error) == len(velocity) == 3

    # calculate_duration = lambda distance, speed: abs(float(distance) / speed)
    def calculate_duration(distance, speed):
        return abs(float(distance) / speed)

    durations = [
        calculate_duration(ee, vv) if (abs(vv) >= zero) else 0.0
        for ee, vv in zip(error, velocity)
    ]

    return max(durations)


def calculate_sync_velocity(error, velocity, max_time, angular_synchronization=False):
    """
    Calculates the synchronized velocity for all velocities to reach their goal
    at the same time. By default, it only synchronizes linear velocities.
    If angular_synchronization is True, then it also synchronizes for angular
    velocities.

    :param error: The distance error tuple in the X, Y and Z axis. It should either
        be three-dimensional (i.e. only linear) or six-dimensional (i.e. linear
        and angular). Its dimension must match that of the velocity argument.
    :type error: list

    :param velocity: Velocity in the X, Y and Z axis. It should either
        be three-dimensional (i.e. only linear) or six-dimensional (i.e. linear
        and angular). Its dimension must match that of the error argument.
    :type velocity: list

    :param max_time: The maximum time required, between the velocities,
    to reach their goal.
    :type max_time: float

    :param angular_synchronization: If True, the twist is synchronized for linear
        and angular velocities. Otherwise the twist is only synchronized for linear
        velocities.
    :type angular_synchronization: bool

    :return: The synchronized velocities.
    :rtype: list

    """
    if angular_synchronization:
        assert len(error) == len(velocity) == 6
    else:
        assert len(error) == len(velocity) == 3

    # A velocity is computed to cover a distance (dist) in a given time (max_time),
    # where max_time is the same for all distances.
    # synchronize_velocity = lambda dist, vel: abs(float(dist) / max_time) * cmp(vel, 0)
    def synchronize_velocity(dist, vel):
        return abs(float(dist) / max_time) * cmp(vel, 0)

    return [
        synchronize_velocity(ee, vv) if (max_time and vv) else 0.0
        for ee, vv in zip(error, velocity)
    ]
