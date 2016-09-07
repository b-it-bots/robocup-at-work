#!/usr/bin/env python
"""
This module contains functions used by the trajectory_generator node.

"""
#-*- encoding: utf-8 -*-

import numpy


def calculate_maximum_duration(point_1, point_2, scaling_factor=1.0):
    """
    Calculates the maximum duration based on the distance between two n-dimensional
    points and the scaling factor.

    :param point_1: An n-dimensional point.
    :type point_1: list

    :param point_2: An n-dimensional point.
    :type point_2: list

    :param scaling_factor: A time scaling factor (in seconds).
    :type scaling_factor: float

    :return: Maximum duration from point_1 to point_2 (in seconds).
    :rtype: float

    """
    assert type(scaling_factor) == float
    assert (type(point_1) == list) or (type(point_1) == tuple),\
        "'point_1' must be of type list, not {0}".format(type(point_1).__name__)
    assert (type(point_2) == list) or (type(point_2) == tuple),\
        "'point_2' must be of type list, not {0}".format(type(point_2).__name__)
    assert len(point_1) == len(point_2),\
        "'point_1' and 'point_2' must be of the same size."

    p1 = numpy.array(point_1)
    p2 = numpy.array(point_2)

    duration = numpy.max(numpy.abs(p1 - p2))
    return duration * scaling_factor


def calculate_discrete_difference(point_1, point_2, maximum_limit=None):
    """
    Calculates the discrete difference between two n-dimensional points and
    limits any element of the difference that exceeds the specified maximum limit.

    :param point_1: An n-dimensional point.
    :type point_1: list

    :param point_2: An n-dimensional point.
    :type point_2: list

    :param maximum_limit: If specified, is an absolute limit for the maximum value
    allowed.
    :type maximum_limit: float or None

    :return: The discrete difference between point_1 to point_2.
    :rtype: list

    """
    assert type(maximum_limit) == float or maximum_limit is None
    assert (type(point_1) == list) or (type(point_1) == tuple),\
        "'point_1' must be of type list, not {0}".format(type(point_1).__name__)
    assert (type(point_2) == list) or (type(point_2) == tuple),\
        "'point_2' must be of type list, not {0}".format(type(point_2).__name__)
    assert len(point_1) == len(point_2),\
        "'point_1' and 'point_2' must be of the same size."

    p1 = numpy.array(point_1)
    p2 = numpy.array(point_2)

    difference = p2 - p1

    if maximum_limit is None:
        return difference.tolist()
    else:
        return [
            dd if abs(dd) <= maximum_limit else maximum_limit * numpy.sign(dd)
            for dd in difference
        ]
