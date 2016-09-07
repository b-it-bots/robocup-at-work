#!/usr/bin/env python
"""
This component computes a shifted pose, by a specified offset, from a input pose.

"""
#-*- encoding: utf-8 -*-

import copy
import itertools
import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf
from dynamic_reconfigure.server import Server
#from pose_shifter_node.cfg import LinearOffsetConfig
import mcr_common_converters.cfg.LinearOffsetConfig as LinearConfig

class PoseShifter(object):
    """
    Computes a shifted pose, by a specified offset, from a input pose.

    """
    def __init__(self):
        # Params
        self.event = None
        self.pose_in = None
        self.listener = tf.TransformListener()

        # Linear offset in X, Y and Z (in meters)
        self.linear_offset = [0,0,0]
        
        print "linear_offset value see::", self.linear_offset
        

        self.reference_frame = rospy.get_param('~reference_frame', None)

        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.pose_out = rospy.Publisher("~pose_out", geometry_msgs.msg.PoseStamped, queue_size=1)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~pose_in", geometry_msgs.msg.PoseStamped, self.pose_in_cb
        )
        # Dynamic Server Reconfiguration
        dynamic_reconfig_srv = Server(LinearConfig, self.dynamic_reconfig_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def pose_in_cb(self, msg):
        """
        Obtains the pose.

        """
        self.pose_in = msg

    def dynamic_reconfig_cb(self,config,level):
                
        self.linear_offset[0] = config.linear_offset_x
        self.linear_offset[1] = config.linear_offset_y
        self.linear_offset[2] = config.linear_offset_z
        return config
        
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
            self.event_out.publish('e_stopped')
            self.reset_inputs()
            return 'INIT'
        elif self.pose_in:
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
            self.event_out.publish('e_stopped')
            self.reset_inputs()
            return 'INIT'
        else:
            pose = self.compute_shifted_pose(self.pose_in)
            if pose is not None:
                if self.reference_frame:
                    pose = self.transform_pose(self.reference_frame, pose)
                self.pose_out.publish(pose)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

        self.reset_inputs()
        return 'IDLE'

    def reset_inputs(self):
        """
        Resets the component's inputs.

        """
        self.event = None
        self.pose_in = None

    def compute_shifted_pose(self, pose_in):
        """
        Computes the shifted pose from 'pose_in' given the specified offset.

        :param pose_in: The pose to be used as reference for the shifted pose.
        :type pose_in: geometry_msgs.msg.PoseStamped

        :return: A shifted pose.
        :rtype: geometry_msgs.msg.PoseStamped

        """
        pose_out = copy.deepcopy(pose_in)
        
        pose_out.pose.position.x += self.linear_offset[0]
        pose_out.pose.position.y += self.linear_offset[1]
        pose_out.pose.position.z += self.linear_offset[2]

        return pose_out

    
    def transform_pose(self, reference_frame, target_pose):
        """
        Transforms the target pose into the reference_frame

        :param reference_frame: The reference frame.
        :type reference_frame: string

        :param target_pose: The current pose.
        :type target_pose: geometry_msgs.msg.PoseStamped

        :return: The target pose transformed to the frame of the reference frame.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        try:
            target_pose.header.stamp = self.listener.getLatestCommonTime(
                target_pose.header.frame_id, reference_frame
            )

            self.listener.waitForTransform(
                target_pose.header.frame_id, reference_frame,
                target_pose.header.stamp, rospy.Duration(self.wait_for_transform)
            )

            transformed_pose = self.listener.transformPose(
                reference_frame, target_pose,
            )

            return transformed_pose

        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None



def main():
    rospy.init_node('pose_shifter', anonymous=True)
    pose_shifter = PoseShifter()
    pose_shifter.start()
