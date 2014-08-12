#!/usr/bin/env python
"""
This module contains functions to extract messages relevant to manipulation.

"""
#-*- encoding: utf-8 -*-


def extract_positions(joint_state, joint_names):
    """
    Extracts the joint position values of the specified joints.

    :type joint_state: sensor_msgs.msg.JointState
    :type joint_names: String[]

    """
    position = []

    for name in joint_names:
        for i, joint in enumerate(joint_state.name):
            if joint == name:
                position.append(joint_state.position[i])

    return position