#!/usr/bin/env python
"""
This component computes the pose of a frame with respect to a specified reference
frame and then shifts it by a specified offset.

It uses the following nodes:
  * (mcr_common_converters) `transform_to_pose_converter`.
  * (mcr_common_converters) `pose_shifter`.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

"""
#-*- encoding: utf-8 -*-

import rospy
import std_msgs.msg


class PoseShifterPipeline(object):
    """
    Components that compute the pose of a frame with respect to a specified reference
    frame and then shifts it by a specified offset.

    """
    def __init__(self):
        # Params
        self.started_components = False
        self.event = None
        self.pose_shifter_status = None

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.start_transformer = rospy.Publisher(
            '~start_transformer', std_msgs.msg.String,
            latch=True , queue_size=1
        )
        self.start_pose_shifter = rospy.Publisher(
            '~start_pose_shifter', std_msgs.msg.String,
            latch=True, queue_size=1
        )

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~pose_shifter_status", std_msgs.msg.String, self.pose_shifter_status_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def pose_shifter_status_cb(self, msg):
        """
        Obtains the status of the pose shifter (as an event).

        """
        self.pose_shifter_status = msg.data

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
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
            return 'RUNNING'
        else:
            return 'INIT'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        self.toggle_components(self.event)

        if self.event == 'e_stop':
            status = 'e_stopped'
            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        if self.pose_shifter_status == 'e_failure':
            status = 'e_failure'
            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        if self.pose_shifter_status == 'e_success':
            status = 'e_success'
            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        else:
            return 'RUNNING'

    def toggle_components(self, event):
        """
        Starts or stops the necessary components based on the event.

        :param event: The event that determines either to start or stop the components.
        :type event: str

        """
        if event == 'e_stopped' or event == 'e_failure' or event == 'e_success':
            self.start_transformer.publish('e_stop')
            self.start_pose_shifter.publish('e_stop')
            self.started_components = False

        if event == 'e_start' and not self.started_components:
            self.start_transformer.publish('e_start')
            self.start_pose_shifter.publish('e_start')
            self.started_components = True

    def reset_component_data(self, result):
        """
        Clears the data of the component.

        :param result: The result of the component, e.g. stopped, failure, success.
        :type result: str

        """
        self.toggle_components(result)
        self.event = None
        self.pose_shifter_status = None
        self.started_components = False


def main():
    rospy.init_node("pose_shifter_pipeline", anonymous=True)
    pose_shifter_pipeline = PoseShifterPipeline()
    pose_shifter_pipeline.start()
