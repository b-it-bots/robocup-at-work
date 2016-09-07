#!/usr/bin/env python
"""
This module contains a component that takes in a list of poses
and on each trigger it selects and publishes one pose from the lists.

"""
# -*- encoding: utf-8
# -*-__author__ = 'shehzad'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import math
import numpy


class PoseSelector(object):
    """
    Stores list of poses and publishes a pose on each trigger.

    """
    def __init__(self):
        # params
        self.monitor_event = None
        self.poses_list = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # publishers
        self.selected_pose_pub = rospy.Publisher(
            '~pose_out', geometry_msgs.msg.PoseStamped, queue_size=1
        )
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=1)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~pose_list_in', geometry_msgs.msg.PoseArray, self.poses_list_cb)

    def start(self):
        """
        Starts the pose selector.

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
        Obtains an event for the pose selector.

        """
        self.monitor_event = msg.data

    def poses_list_cb(self, msg):
        """
        Obtains the first pose.

        """
        self.poses_list = msg

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.poses_list:
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        elif self.monitor_event == 'e_start':
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """

        selected_pose = self.select_pose()
        if selected_pose:
            self.selected_pose_pub.publish(selected_pose)
            self.event_out.publish('e_success')
        else:
            self.event_out.publish('e_failure')

        self.monitor_event = None

        return 'IDLE'

    def select_pose(self):

        if not self.poses_list.poses:
            return None

        pose_msg = geometry_msgs.msg.PoseStamped()

        pose_msg.header.frame_id = self.poses_list.header.frame_id
        pose_msg.header.stamp = self.poses_list.header.stamp
        pose_msg.pose = self.poses_list.poses.pop()

        return pose_msg

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.monitor_event = None
        self.poses_list = None


def main():
    rospy.init_node('pose_selector_node', anonymous=True)
    pose_selector = PoseSelector()
    pose_selector.start()
