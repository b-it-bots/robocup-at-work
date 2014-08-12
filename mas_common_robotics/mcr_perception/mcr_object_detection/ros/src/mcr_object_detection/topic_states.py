#!/usr/bin/env python

PACKAGE = 'mcr_object_detection'

import roslib
roslib.load_manifest(PACKAGE)

import rospy

import smach
import std_msgs.msg
import mcr_perception_msgs.msg

class FindWorkspace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                        output_keys=['polygon'],
                        outcomes=['succeeded', 'aborted'])
        self.event_pub = rospy.Publisher('/mcr_perception/workspace_finder/event_in', std_msgs.msg.String, queue_size=1)
        self.event_sub = rospy.Subscriber('/mcr_perception/workspace_finder/event_out', std_msgs.msg.String, self.e_callback)
        self.polygon_sub = rospy.Subscriber('/mcr_perception/workspace_finder/polygon', mcr_perception_msgs.msg.PlanarPolygon, self.poly_callback)

    def e_callback(self, event):
        self.polygon_event = event.data

    def poly_callback(self, data):
        self.polygon = data
        
    def execute(self, userdata):
        self.polygon_event = ""
        self.polygon = None

        self.event_pub.publish("e_trigger")

        timeout = rospy.Duration.from_sec(10.0) # wait max of 10.0 seconds
        start_time = rospy.Time.now()
        
        while (True):
            if self.polygon_event == "e_done" and self.polygon is not None:
                userdata.polygon = self.polygon
                return 'succeeded'

            if self.polygon_event == "e_failed":
                return 'aborted'

            if (rospy.Time.now() - start_time) > timeout:
                rospy.logerr('Timeout of %f seconds exceeded for FindWorkspace' % float(timeout.to_sec()))
                return 'aborted'

            rospy.sleep(0.01)
            
        return 'succeeded'
