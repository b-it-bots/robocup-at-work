#!/usr/bin/env python
"""
This module contains functions to limit a variable
(e.g. it can be used to limit a velocity).

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'


def limit_value(value, limit):
    """
    Limits a value if it is greater than the limit, or lower
    than the negative limit.

    :param value: The input value.
    :type value: float

    :param limit: Maximum and minimum (as negative maximum) value allowed.
    :type limit: float

    :return: The limited value if exceeds the specified limit, or the input value.
    :rtype: float

    """
    if value > limit:
        limited_value = limit
    elif value < -limit:
        limited_value = -limit
    else:
        return value

    return limited_value
