#!/usr/bin/env python
"""
This module contains functions used by the simple_pregrasp_planner node.

"""
#-*- encoding: utf-8 -*-


def brics_joint_positions_to_list(joint_configuration):
    """
    Converts a BRICS message of joint positions into a list of real values.

    :param joint_configuration: Joint configuration as a BRICS message.
    :type joint_configuration: brics_actuator.msg.JointPositions

    :return: A list of real values representing the joints as specified
             by the BRICS message.
    :rtype: list

    """
    return [joint.value for joint in joint_configuration.positions]
