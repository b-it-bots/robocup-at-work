#!/usr/bin/env python
"""
This component publishes a PoseStamped message based on the
input pose

"""
#-*- encoding: utf-8 -*-

import rospy
import geometry_msgs.msg


class PosePublisher(object):
    """
    Publishes a PoseStamped message.

    """
    def __init__(self):
        # params
        self.desired_pose = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # publishers
        self.pose_out = rospy.Publisher('~pose_out', geometry_msgs.msg.PoseStamped, queue_size=1)
        # subscribers
        rospy.Subscriber("~pose_in", geometry_msgs.msg.PoseStamped, self.pose_in_cb)

    def pose_in_cb(self, msg):
        """
        Obtains a pose for the component.

        """
        self.desired_pose = msg

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")

        while not rospy.is_shutdown():
            if self.desired_pose:
                self.desired_pose.header.stamp = rospy.Time.now()
                self.pose_out.publish(self.desired_pose)
            self.loop_rate.sleep()


def main():
    rospy.init_node("pose_publisher", anonymous=True)
    pose_publisher = PosePublisher()
    pose_publisher.start()
