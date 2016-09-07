#!/usr/bin/env python
"""
This module contains a component that computes pose from given pose error.

"""
#-*- encoding: utf-8 -*-
__author__ = 'padmaja'

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import mcr_manipulation_msgs.msg
import tf

class RelativeDisplacementCalculator(object):

    def __init__(self):
        # params
        self.monitor_event = None
        # node cycle rate (in Hz)
        self.loop_rate = rospy.get_param('~loop_rate', 10)
        self.reference_frame = rospy.get_param('~reference_frame', '/base_link')
        self.pose_error = None
        self.r = rospy.Rate(self.loop_rate) 
        self.listener = tf.TransformListener()

        # publishers
        self.pose_pub = rospy.Publisher('~pose', PoseStamped)
        self.event_out_pub = rospy.Publisher('~event_out', String)
        
        # subscribers
        rospy.Subscriber('~event_in', String, self.event_in_cb)
        rospy.Subscriber('~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference, self.pose_error_cb)

    def start(self):
        """
        Starts relative displacement calculator calculator.
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
            self.r.sleep()

    def event_in_cb(self, msg):
        """
        Obtains an event for the pose computation.
        """
        self.monitor_event = msg.data

    def pose_error_cb(self, msg):
        """
        Stores pose error.
        """
        self.pose_error = msg

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str
        """
        if self.pose_error:
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str
        """
        if self.monitor_event == 'e_start':
            self.monitor_event = None
            return 'RUNNING'
        elif self.monitor_event == 'e_stop':
            self.monitor_event = None
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str
        """
        if self.monitor_event == 'e_stop':
            self.monitor_event = None
            self.pose_error = None
            return 'INIT'
        else:
            pose_from_pose_error = self.relative_displacement_calculator()
            if pose_from_pose_error:
                self.pose_pub.publish(pose_from_pose_error)
                event_out_msg = String()
                event_out_msg.data = 'e_done'
                self.event_out_pub.publish(event_out_msg)
            return 'RUNNING'

    def relative_displacement_calculator(self):
        """
        Computes relative displacement from pose error.

        :return: pose from pose error.
        :rtype: geometry_msgs.msg.PoseStamped
        """
        pose_from_pose_error = PoseStamped()
        pose_from_pose_error.header = self.pose_error.header
        pose_from_pose_error.pose.position.x = self.pose_error.linear.x 
        pose_from_pose_error.pose.position.y = self.pose_error.linear.y 
        pose_from_pose_error.pose.position.z = self.pose_error.linear.z

        try:
            self.listener.waitForTransform(
                self.reference_frame, self.pose_error.header.frame_id, 
                self.pose_error.header.stamp, rospy.Duration(0.1)
            )

            transformed_pose = self.listener.transformPose(
                self.reference_frame, pose_from_pose_error
            )

            return transformed_pose

        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None

def main():
    rospy.init_node('relative_displacement_calculator', anonymous=True)
    relative_displacement_calculator = RelativeDisplacementCalculator()
    relative_displacement_calculator.start()
