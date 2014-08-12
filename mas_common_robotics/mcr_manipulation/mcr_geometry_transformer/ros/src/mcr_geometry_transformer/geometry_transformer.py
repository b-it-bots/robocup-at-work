#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
import geometry_transformer_util


class GeometryTransformer:

    def __init__(self):
        self.listener = tf.TransformListener()


    def transformWrench(self, wrench_in, target_frame = "base_link"):
        '''
        Transform the provided wrench to the target frame. Throws an exception
        if the transformation failed.
        
        :param wrench_in: The wrench which should be transformed.
        :type wrench_in: geometry_msgs.msg.WrenchStamped]
        
        :param target_frame: The frame into which the wrench should be
        transformed.
        :type target_frame: String
        
        :return: The transformed wrench.
        :rtype: geometry_msgs.msg.WrenchStamped
        '''
        self.listener.waitForTransform(target_frame, wrench_in.header.frame_id,
                                       wrench_in.header.stamp,
                                       rospy.Duration(5.0))
        t = self.listener.asMatrix(target_frame, wrench_in.header)
        
        wrench_out = geometry_msgs.msg.WrenchStamped()
        wrench_out = geometry_transformer_util.transform_wrench(t, wrench_in)
        wrench_out.header.frame_id = target_frame
        wrench_out.header.stamp = wrench_in.header.stamp
        
        return wrench_out
