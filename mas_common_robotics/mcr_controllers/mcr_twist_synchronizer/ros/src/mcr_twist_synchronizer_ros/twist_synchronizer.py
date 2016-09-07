#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This component synchronizes the velocities of a twist (represented as a
geometry_msgs/TwistStamped message), such that each component of a Cartesian error
(compensated by the twist's velocities) simultaneously reaches zero.

**Input(s):**
  * `twist`: The twist to be synchronized.
  * `pose_error`: The component-wise Cartesian difference (error).

**Output(s):**
  * `synchronized_twist`: The synchronized twist.

**Parameter(s):**
  * `angular_synchronization`: If True, it also synchronizes the angular and linear
  velocities. By default, it only synchronizes the linear velocities (bool).
  * `near_zero`: A value to prevent division by near-zero values.
  * `loop_rate`: Node cycle rate (in hz).

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg
import mcr_twist_synchronizer_ros.twist_synchronizer_utils as utils


class TwistSynchronizer(object):
    """
    Synchronizes a twist.

    """
    def __init__(self):
        # Params
        self.event = None
        self.twist = None
        self.pose_error = None

        # If True, it also synchronizes the angular and linear velocities.
        # By default, it only synchronizes the linear velocities.
        self.angular_synchronization = rospy.get_param('~angular_synchronization', False)

        # A value to prevent division by near-zero values.
        self.near_zero = rospy.get_param('~near_zero', 0.001)

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=5)
        self.synchronized_twist = rospy.Publisher(
            '~synchronized_twist', geometry_msgs.msg.TwistStamped, queue_size=5
        )

        # Subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~twist', geometry_msgs.msg.TwistStamped, self.twist_cb)
        rospy.Subscriber(
            '~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference,
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

    def twist_cb(self, msg):
        """
        Obtains the twist.

        """
        self.twist = msg

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
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        elif self.twist and self.pose_error:
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
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            synchronized_twist = self.synchronize_twist()
            if synchronized_twist:
                self.synchronized_twist.publish(synchronized_twist)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

            self.reset_component_data()
            return 'IDLE'

    def synchronize_twist(self):
        """
        Synchronizes a twist to make its velocities finish at the same time.

        :return: The synchronized twist.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        synchronized_twist = geometry_msgs.msg.TwistStamped()
        synchronized_twist.header.frame_id = self.twist.header.frame_id
        synchronized_twist.header.stamp = self.twist.header.stamp

        if self.angular_synchronization:
            error = [
                self.pose_error.linear.x, self.pose_error.linear.y,
                self.pose_error.linear.z, self.pose_error.angular.x,
                self.pose_error.angular.y, self.pose_error.angular.z
            ]

            velocity = [
                self.twist.twist.linear.x, self.twist.twist.linear.y,
                self.twist.twist.linear.z, self.twist.twist.angular.x,
                self.twist.twist.angular.y, self.twist.twist.angular.z
            ]
        else:
            error = [
                self.pose_error.linear.x, self.pose_error.linear.y,
                self.pose_error.linear.z
            ]

            velocity = [
                self.twist.twist.linear.x, self.twist.twist.linear.y,
                self.twist.twist.linear.z
            ]

        # Calculate maximum time to reach the goal.
        max_time = utils.calculate_max_time(
            error, velocity, self.angular_synchronization, self.near_zero
        )

        # Calculate the velocities to reach the goal at the same time.
        sync_velocities = utils.calculate_sync_velocity(
            error, velocity, max_time, self.angular_synchronization
        )

        synchronized_twist.twist.linear.x = sync_velocities[0]
        synchronized_twist.twist.linear.y = sync_velocities[1]
        synchronized_twist.twist.linear.z = sync_velocities[2]
        if self.angular_synchronization:
            synchronized_twist.twist.angular.x = sync_velocities[3]
            synchronized_twist.twist.angular.y = sync_velocities[4]
            synchronized_twist.twist.angular.z = sync_velocities[5]

        return synchronized_twist

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.twist = None
        self.pose_error = None


def main():
    rospy.init_node('twist_synchronizer', anonymous=True)
    twist_synchronizer = TwistSynchronizer()
    twist_synchronizer.start()
