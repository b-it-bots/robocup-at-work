#!/usr/bin/env python
"""
This module contains a component that converts a PoseStamped
message into a ROS-TF transform.

"""
#-*- encoding: utf-8 -*-
__author__ = 'nkorip2s'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf


class PoseToTransformConverter(object):
    """
    Converts pose to transform. First converts the received pose
    from received frame to the reference frame. Then broadcasts
    the converted pose as a transform.

    """
    def __init__(self):
        # variables for callbacks
        self.event_in = None
        self.pose_in = None

        # variable to keep output transform in memory 
        self.transformed_pose = None

        self.is_transformation_success = False
        self.is_previously_transformed = False
        self.is_continuously_transformed = rospy.get_param('~is_continuously_transformed', False)

        self.reference_frame = rospy.get_param('~reference_frame', None)
        assert self.reference_frame is not None, "Reference frame must be defined."

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        self.broadcast_transform_name = rospy.get_param('~broadcast_transform_name', None)
        assert self.broadcast_transform_name is not None, "Broadcast transform name must be defined."

        # tf listener for looking up which frame the frame ID is pointing to
        self.tf_listener = tf.TransformListener()

        # tf broadcaster for finally broadcasting the converted tf
        self.tf_broadcaster = tf.TransformBroadcaster()

        # subscriber for listening for event
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)

        # subscriber for listening for input pose stamped message 
        rospy.Subscriber('~input_pose', geometry_msgs.msg.PoseStamped, self.data_in_cb)

        # publisher for event_out
        self.event_out_publisher = rospy.Publisher('~event_out', std_msgs.msg.String)

    def start_pose_to_transform_converter(self):
        """
        Starts the pose to transform converter. Keeps looping through the states
        depending on events received and data received.

        """
        rospy.loginfo("Ready to start...")
        
        state = 'INIT'
        rospy.loginfo("STATE : INIT")
        
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
        Obtains an event for the pose to transform converter.

        """
        self.event_in = msg.data

    def data_in_cb(self, msg):
        """
        Obtains PoseStamped msg

        """
        self.pose_in = msg

    def init_state(self):
        """
        Executes the INIT state of the state machine.
        When e_start event is received, it changes state from IDLE to INIT

        :return: The updated state.
        :rtype: str

        """
        if self.event_in == 'e_start':
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.
        If data is received then moves from IDLE to RUNNING state.
        If e_stop event is received moves from IDLE to INIT (reset to defaults)
        Else stay in IDLE state

        :return: The updated state.
        :rtype: str

        """
        if self.event_in == 'e_stop':
            self.reset_to_defaults()
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
        if self.event_in == 'e_stop':
            self.reset_to_defaults()
            return 'INIT'
        else:
            # generate transformed pose at least once
            if not self.is_previously_transformed:
                self.is_transformation_success = self.generate_transformed_pose()
                self.is_previously_transformed = True
            # generate transformed pose always if the flag is set
            elif self.is_continuously_transformed:
                self.is_transformation_success = self.generate_transformed_pose()
            
            if self.is_transformation_success:
                self.broadcast_converted_transform()
                self.event_out_publisher.publish('e_success')
                return 'RUNNING'
            else:
                self.event_out_publisher.publish('e_failure')
                self.reset_to_defaults()
                return 'INIT'
            

    def generate_transformed_pose(self):
        """
        Generates a transformed pose from the recieved frame and reference frame
        and stores it as a class variable.

        """
        received_frame = self.pose_in.header.frame_id

        try:
            self.tf_listener.waitForTransform(
                received_frame, self.reference_frame,
                self.pose_in.header.stamp, rospy.Duration(self.wait_for_transform)
            )
            self.transformed_pose = self.tf_listener.transformPose(self.reference_frame, self.pose_in)
            return True
        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return False

    def broadcast_converted_transform(self):
        """
        Publishes the converted transform based on the transformed pose
        between from received frame w.r.t. reference frame.

        """
        p = self.transformed_pose.pose.position
        q = self.transformed_pose.pose.orientation

        self.tf_broadcaster.sendTransform((p.x, p.y, p.z), (q.x, q.y, q.z,q.w),
                              rospy.Time.now(), self.broadcast_transform_name,
                              self.transformed_pose.header.frame_id)

    def reset_to_defaults(self):
        self.event_in = None
        self.pose_in = None
        self.transformed_pose = None
        self.is_transformation_success = False
        self.is_previously_transformed = False



def main():
    rospy.init_node('transform_to_pose_converter', anonymous=True)
    pose_to_transform_converter = PoseToTransformConverter()
    pose_to_transform_converter.start_pose_to_transform_converter()