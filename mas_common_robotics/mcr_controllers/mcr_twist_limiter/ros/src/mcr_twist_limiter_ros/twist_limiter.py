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

        # node cycle time (in seconds)
        self.cycle_time = rospy.get_param('~cycle_time')

        # publishers
        self.limited_twist = rospy.Publisher(
            '~limited_twist', geometry_msgs.msg.TwistStamped
        )

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
            rospy.sleep(self.cycle_time)

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
        if self.twist:
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
            return 'RUNNING'
        elif self.event == 'e_stop':
            self.twist = None
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.twist = None
            return 'INIT'
        else:
            limited_twist = self.limit_twist()
            self.limited_twist.publish(limited_twist)

            return 'RUNNING'

    def limit_twist(self):
        """
        Limits a twist if it exceeds the specified maximum.

        :return: The Cartesian velocity to reduce the position error.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        limited_twist = geometry_msgs.msg.TwistStamped()
        limited_twist.header.frame_id = self.twist.header.frame_id
        limited_twist.header.stamp = self.twist.header.stamp

        dimensions = ['x', 'y', 'z']
        linear_velocities = [
            self.max_velocity_x, self.max_velocity_y, self.max_velocity_z
        ]
        angular_velocities = [
            self.max_velocity_roll, self.max_velocity_pitch, self.max_velocity_yaw
        ]

        for i, dim in enumerate(dimensions):
            linear_velocity = limit_velocity(
                getattr(self.twist.twist.linear, dim), linear_velocities[i]
            )
            angular_velocity = limit_velocity(
                getattr(self.twist.twist.angular, dim), angular_velocities[i]
            )

            setattr(limited_twist.twist.linear, dim, linear_velocity)
            setattr(limited_twist.twist.angular, dim, angular_velocity)

        return limited_twist


def limit_velocity(velocity, max_velocity):
    """
    Limits a velocity if it exceeds the specified maximum.

    :param velocity: The input velocity.
    :type velocity: float

    :param max_velocity: Maximum velocity allowed.
    :type max_velocity: float

    :return: The limited velocity if exceeds the specified maximum,
             or the input velocity.
    :rtype: float

    """
    if velocity > max_velocity:
        limited_velocity = max_velocity
    elif velocity < -max_velocity:
        limited_velocity = -max_velocity
    else:
        return velocity

    return limited_velocity


def main():
    rospy.init_node('twist_limiter', anonymous=True)
    twist_limiter = TwistLimiter()
    twist_limiter.start()
