#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This module contains a component that monitors stores the poses of the
cavities and publishes the pose of cavity in which the object can be put in, on trigger.

**Input(s):**
  * cavity: cavity message for the identified cavities.
  * object_name: Name of the object for which the cavity pose id needed.
  * event : To trigger the run state of the node. (it assumes that teh object pose
    is already published.)

**Output(s):**
  * `cavity_pose`: The pose of the cavity.

**Relevant parameter(s):**
  * loop_rate : Loop rate of the node.
"""

import rospy
import mcr_perception_msgs.msg as mpm
import geometry_msgs.msg
import std_msgs.msg
import numpy as np

__author__ = 'padmaja'


class CavityPoseSelector(object):
    """
    Publishes the pose of the cavity in which the requested object can be dropped.

    """
    def __init__(self):
        # params
        self.event = None
        self.cavity_msg_array = []
        self.object_name = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # publishers
        self.cavity_pose_pub = rospy.Publisher(
            "~cavity_pose", geometry_msgs.msg.PoseStamped, queue_size=1)
        self.event_out_pub = rospy.Publisher(
            "~event_out", std_msgs.msg.String, queue_size=1)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~cavity", mpm.Cavity, self.cavity_cb)
        rospy.Subscriber("~object_name", std_msgs.msg.String, self.object_name_cb)

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

    def cavity_cb(self, msg):
        """
        Obtains the first pose.

        """
        rospy.loginfo("Cavity received: {0}".format(msg.name))
        self.cavity_msg_array.append(msg)

    def object_name_cb(self, msg):
        """
        Obtains an event for the component.

        """
        rospy.loginfo("Object received: {0}".format(msg.data))
        self.object_name = msg
        self.old_object_name = msg

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
        if len(self.cavity_msg_array) > 0:
            return 'RUNNING'
        elif self.event == 'e_stop':
            self.object_name = None
            self.old_object_name = None
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
        if self.event == 'e_stop':
            self.object_name = None
            self.old_object_name = None
            return 'INIT'
        elif self.event == 'e_trigger':
            self.object_name = self.old_object_name
            self.publish_event_out()
            self.event = None
            return 'RUNNING'
        else:
            self.publish_event_out()
            return 'RUNNING'

    def publish_event_out(self):
        if self.object_name is not None:
            found_cavity = 'e_failure'
            cavity_name = rospy.get_param('~' + self.object_name.data, None)
            if cavity_name:
                for idx, cavity in enumerate(self.cavity_msg_array):
                    if cavity.name == cavity_name:
                        rospy.loginfo("Cavity selected: {0}".format(cavity_name))
                        self.cavity_pose_pub.publish(cavity.pose)
                        found_cavity = 'e_success'
                        break
            self.object_name = None
            e_out = std_msgs.msg.String()
            e_out.data = found_cavity
            self.event_out_pub.publish(e_out)


def main():
    rospy.init_node('cavity_pose_selector', anonymous=True)
    cavity_pose_selector = CavityPoseSelector()
    cavity_pose_selector.start()
