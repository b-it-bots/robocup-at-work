#!/usr/bin/env python
"""
This module contains a component that sets the individual components
of a twist (represented as a geometry_msgs/TwistStamped message) to
a specified maximum, if they exceed their respective limit.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_twist_limiter.limiter as limiter


class TwistLimiter(object):
    """
    Sets the individual components of a twist to a specified
    maximum, if they exceed their respective limit.

    """
    def __init__(self):
        # params
        self.event = None
        self.twist = None

        # Maximum linear velocities in the axes X, Y and Z (in meters/second)
        self.max_velocity_x = rospy.get_param('~max_velocity_x', 0.1)
        self.max_velocity_y = rospy.get_param('~max_velocity_y', 0.1)
        self.max_velocity_z = rospy.get_param('~max_velocity_z', 0.1)

        # Maximum angular velocities around the axes X, Y and Z (in radians/second)
        self.max_velocity_roll = rospy.get_param('~max_velocity_roll', 0.1)
        self.max_velocity_pitch = rospy.get_param('~max_velocity_pitch', 0.1)
        self.max_velocity_yaw = rospy.get_param('~max_velocity_yaw', 0.1)

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # publishers
        self.limited_twist = rospy.Publisher(
            '~limited_twist', geometry_msgs.msg.TwistStamped, queue_size=5
        )
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=5)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~twist', geometry_msgs.msg.TwistStamped, self.twist_cb)

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def twist_cb(self, msg):
        """
        Obtains the twist.

        """
        self.twist = msg

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event = None
            self.twist = None
            return 'INIT'
        elif self.twist:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event = None
            self.twist = None
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            limited_twist = self.limit_twist()
            if limited_twist:
                self.limited_twist.publish(limited_twist)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

            self.event = None
            self.twist = None
            return 'IDLE'

    def limit_twist(self):
        """
        Limits a twist if it exceeds the specified maximum.

        :return: The Cartesian velocity to reduce the position error.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        limited_twist = geometry_msgs.msg.TwistStamped()
        limited_twist.header.frame_id = self.twist.header.frame_id
        limited_twist.header.stamp = self.twist.header.stamp

        limited_twist.twist.linear.x = limiter.limit_value(
            self.twist.twist.linear.x, self.max_velocity_x
        )
        limited_twist.twist.linear.y = limiter.limit_value(
            self.twist.twist.linear.y, self.max_velocity_y
        )
        limited_twist.twist.linear.z = limiter.limit_value(
            self.twist.twist.linear.z, self.max_velocity_z
        )
        limited_twist.twist.angular.x = limiter.limit_value(
            self.twist.twist.angular.x, self.max_velocity_roll
        )
        limited_twist.twist.angular.y = limiter.limit_value(
            self.twist.twist.angular.y, self.max_velocity_pitch
        )
        limited_twist.twist.angular.z = limiter.limit_value(
            self.twist.twist.angular.z, self.max_velocity_yaw
        )

        return limited_twist


def main():
    rospy.init_node('twist_limiter', anonymous=True)
    twist_limiter = TwistLimiter()
    twist_limiter.start()
