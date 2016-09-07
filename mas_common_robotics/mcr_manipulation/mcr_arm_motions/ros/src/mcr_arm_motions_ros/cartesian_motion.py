#!/usr/bin/env python
"""
This module contains a component that publishes a Cartesian velocity.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg


class CartesianMotion(object):
    """
    Publishes a Cartesian velocity based on an event received,
    e.g. 'e_start', 'e_stop'. If 'e_stop' is received, it publishes
    a Cartesian velocity with all its components equal to zero.

    """
    def __init__(self):
        # params
        self.event = None
        self.desired_velocity = None
        self.start_time = None
        self.time_out = False

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # the duration of the arm motion (in seconds)
        self.motion_duration = rospy.get_param('~motion_duration', 0.5)

        # publishers
        self.event_out = rospy.Publisher(
            "~event_out", std_msgs.msg.String, queue_size=1)
        self.velocity_command = rospy.Publisher(
            "~velocity_command", geometry_msgs.msg.TwistStamped, queue_size=1)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String,
                         self.event_in_cb)
        rospy.Subscriber("~desired_velocity",
                         geometry_msgs.msg.TwistStamped,
                         self.desired_velocity_cb)

    def start(self):
        """
        Starts the Cartesian motion.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():
            if self.start_time:
                duration = rospy.Time.now() - self.start_time
                if duration >= rospy.Duration.from_sec(self.motion_duration):
                    self.time_out = True

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def desired_velocity_cb(self, msg):
        """
        Obtains the desired Cartesian velocity.

        """
        self.desired_velocity = msg

    def event_in_cb(self, msg):
        """
        Obtains an event for the Cartesian motion.

        """
        self.event = msg.data

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.desired_velocity:
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            self.start_time = rospy.Time.now()
            return 'RUNNING'
        elif self.event == 'e_stop':
            self.time_out = False
            self.start_time = None
            self.desired_velocity = None
            self.event = None
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop' or self.time_out:
            self.set_velocity_to_zero()
            self.velocity_command.publish(self.desired_velocity)
            self.event_out.publish('e_stop')
            if self.time_out:
                self.event_out.publish('e_timed_out')

            self.time_out = False
            self.start_time = None
            self.desired_velocity = None
            self.event = None
            return 'INIT'
        else:
            self.velocity_command.publish(self.desired_velocity)
            return 'RUNNING'

    def set_velocity_to_zero(self):
        """
        Sets each of the Cartesian velocity's components equal to zero.

        """
        self.desired_velocity.twist.linear.x = 0.0
        self.desired_velocity.twist.linear.y = 0.0
        self.desired_velocity.twist.linear.z = 0.0
        self.desired_velocity.twist.angular.x = 0.0
        self.desired_velocity.twist.angular.y = 0.0
        self.desired_velocity.twist.angular.z = 0.0


def main():
    rospy.init_node("cartesian_motion", anonymous=True)
    cartesian_motion = CartesianMotion()
    cartesian_motion.start()
