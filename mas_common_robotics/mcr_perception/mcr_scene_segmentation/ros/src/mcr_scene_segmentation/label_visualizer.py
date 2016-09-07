#!/usr/bin/env python


import rospy

from visualization_msgs.msg import Marker, MarkerArray
from color import colors

PACKAGE = 'mcr_scene_segmentation'


class LabelVisualizer:

    def __init__(self, topic_name, color, check_subscribers=True):
        self.marker_pub = rospy.Publisher(topic_name, MarkerArray, queue_size=1)
        self.check_subs = check_subscribers
        self.color = colors[color].to_msg()

    def publish(self, labels, positions, frame_id='/camera_rgb_optical_frame'):
        if self.check_subs and not self.marker_pub.get_num_connections():
            return
        ma = MarkerArray()
        for i, (l, p) in enumerate(zip(labels, positions)):
            m = Marker()
            m.header.frame_id = frame_id
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.scale.z = 0.04
            m.color = self.color
            m.ns = 'labels'
            m.id = i
            m.pose.position.x = p.x
            m.pose.position.y = p.y + 0.02
            m.pose.position.z = p.z
            m.pose.orientation.w = 1.0
            m.text = l
            ma.markers.append(m)
            self.marker_pub.publish(ma)
