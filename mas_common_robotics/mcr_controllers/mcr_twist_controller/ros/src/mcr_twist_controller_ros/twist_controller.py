#!/usr/bin/env python
"""
This module contains a component that calculates a
twist to reduce the difference between two poses.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg
import mcr_algorithms.controller.pid_controller as pid_controller

# This value is used as the 'current_value' for the 'pid_controller'
# from 'mcr_algorithms.controller.pid_controller', since this twist
# controller only receives an error which is used as 'set_value'.
ZERO = 0.0


class TwistController(object):
    """
    Calculates a twist, based on the Cartesian distance
    error between two poses.

    """
    def __init__(self):
        # params
        self.event = None
        self.pose_error = None

        # proportional gains for the Cartesian linear velocities
        self.p_gain_x = rospy.get_param('~p_gain_x', 0.0)
        self.p_gain_y = rospy.get_param('~p_gain_y', 0.0)
        self.p_gain_z = rospy.get_param('~p_gain_z', 0.0)
        self.p_gain_roll = rospy.get_param('~p_gain_roll', 0.0)
        self.p_gain_pitch = rospy.get_param('~p_gain_pitch', 0.0)
        self.p_gain_yaw = rospy.get_param('~p_gain_yaw', 0.0)

        # create controllers
        self.x_controller = pid_controller.p_controller(self.p_gain_x)
        self.y_controller = pid_controller.p_controller(self.p_gain_y)
        self.z_controller = pid_controller.p_controller(self.p_gain_z)
        self.roll_controller = pid_controller.p_controller(self.p_gain_roll)
        self.pitch_controller = pid_controller.p_controller(self.p_gain_pitch)
        self.yaw_controller = pid_controller.p_controller(self.p_gain_yaw)

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # publishers
        self.controlled_twist = rospy.Publisher(
            '~controlled_twist', geometry_msgs.msg.TwistStamped, queue_size=5
        )
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=5)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            '~pose_error',
            mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference,
            self.pose_error_cb
        )

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

    def pose_error_cb(self, msg):
        """
        Obtains the pose error.

        """
        self.pose_error = msg

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
            self.pose_error = None
            return 'INIT'
        elif self.pose_error:
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
            self.pose_error = None
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            cartesian_velocity = self.calculate_cartesian_velocity()
            if cartesian_velocity:
                self.controlled_twist.publish(cartesian_velocity)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

            self.event = None
            self.pose_error = None
            return 'IDLE'

    def calculate_cartesian_velocity(self):
        """
        Calculates a synchronized Cartesian velocity, based on a position error,
        for the three linear velocities and the three angular velocities.

        :return: The Cartesian velocity to reduce the position error.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        cartesian_velocity = geometry_msgs.msg.TwistStamped()
        cartesian_velocity.header.frame_id = self.pose_error.header.frame_id
        cartesian_velocity.header.stamp = rospy.Time.now()

        velocity_x = self.x_controller.control(self.pose_error.linear.x, ZERO)
        velocity_y = self.y_controller.control(self.pose_error.linear.y, ZERO)
        velocity_z = self.z_controller.control(self.pose_error.linear.z, ZERO)
        velocity_roll = self.roll_controller.control(self.pose_error.angular.x, ZERO)
        velocity_pitch = self.pitch_controller.control(self.pose_error.angular.y, ZERO)
        velocity_yaw = self.yaw_controller.control(self.pose_error.angular.z, ZERO)

        cartesian_velocity.twist.linear.x = velocity_x
        cartesian_velocity.twist.linear.y = velocity_y
        cartesian_velocity.twist.linear.z = velocity_z
        cartesian_velocity.twist.angular.x = velocity_roll
        cartesian_velocity.twist.angular.y = velocity_pitch
        cartesian_velocity.twist.angular.z = velocity_yaw

        return cartesian_velocity


def main():
    rospy.init_node('twist_controller', anonymous=True)
    twist_controller = TwistController()
    twist_controller.start()
