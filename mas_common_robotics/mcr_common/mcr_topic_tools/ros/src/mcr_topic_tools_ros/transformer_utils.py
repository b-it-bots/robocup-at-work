#!/usr/bin/env python
"""
This module contains a collection of functions to transform topics, e.g. to convert
a 'brics_actuator/JointPosition' message into a 'sensor_msgs/JointState'.

"""
#-*- encoding: utf-8 -*-

import sensor_msgs.msg


def brics_joints_to_joint_state(m):
    """
    Transforms a PoseStamped message to a Pose, and if rounding is enable
    then the message elements are rounded.

    :param m: The message to be rounded.
    :type m: brics_actuator.msg.JointPosition

    :return: The transformed message.
    :rtype : sensor_msgs.msg.JointState

    """
    message_out = sensor_msgs.msg.JointState()
    # Assumption: The time stamp of the first joint is used as the general time stamp
    message_out.header.stamp = m.positions[0].timeStamp
    message_out.name = [i.joint_uri for i in m.positions]
    message_out.position = [i.value for i in m.positions]

    return message_out
