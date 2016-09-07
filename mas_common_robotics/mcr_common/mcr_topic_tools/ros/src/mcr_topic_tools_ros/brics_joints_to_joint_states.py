#!/usr/bin/env python
"""
This module transforms a brics_actuator.msg.JointPositions message
into a sensor_msgs.msg.JointState message.

"""
#-*- encoding: utf-8 -*-

import rospy
import sensor_msgs.msg
import brics_actuator.msg
import mcr_topic_tools_ros.transformer_utils as topic_utils


class BricsJointsToJointStates(object):
    """
    Transforms a brics_actuator.msg.JointPositions message
    into a sensor_msgs.msg.JointState message.

    """
    def __init__(self):
        # Params
        self.configuration_in = None

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # Publishers
        self.configuration_out = rospy.Publisher(
            '~configuration_out', sensor_msgs.msg.JointState, queue_size=1
        )

        # Subscribers
        rospy.Subscriber(
            '~configuration_in', brics_actuator.msg.JointPositions,
            self.configuration_in_cb
        )

    def start(self):
        """
        Starts the component-wise pose error calculator.

        """
        rospy.loginfo("Ready to start...")

        while not rospy.is_shutdown():
            if self.configuration_in:
                configuration_out = topic_utils.brics_joints_to_joint_state(
                    self.configuration_in
                )
                self.configuration_out.publish(configuration_out)
            else:
                rospy.logdebug("Waiting for data...")

            self.loop_rate.sleep()

    def configuration_in_cb(self, msg):
        """
        Obtains the joint configuration.

        """
        self.configuration_in = msg


def main():
    rospy.init_node('brics_joints_to_joint_states', anonymous=True)
    brics_joints_to_joint_states = BricsJointsToJointStates()
    brics_joints_to_joint_states.start()
