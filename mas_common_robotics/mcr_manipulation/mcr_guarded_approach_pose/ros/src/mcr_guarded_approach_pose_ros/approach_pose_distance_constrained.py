#!/usr/bin/env python
"""
This component moves a manipulator in Cartesian space until a pose is reached.

It uses the following nodes:
  * (mcr_geometric_relation_monitors) component_wise_pose_error_monitor.
  * (mcr_arm_motions) cartesian_motion_node
  * (mcr_guarded_approach_pose) component_wise_pose_error_monitor.
  * (mcr_pose_generator) pose_publisher.
  * (mcr_common_converters) transform_to_pose_converter.
  * (mcr_manipulation_measurers) component_wise_pose_error_calculator.
  * (mcr_arm_motions) twist_publisher.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

"""
#-*- encoding: utf-8 -*-

import rospy
import std_msgs.msg


class ApproachPoseDistanceConstrained(object):
    """
    Components that move the arm until a pose is reached.

    """
    def __init__(self):
        # params
        self.started_components = False
        self.event = None
        self.motion_feedback = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.start_pose_converter = rospy.Publisher(
            '~start_pose_converter', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_calculator = rospy.Publisher(
            '~start_calculator', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_twist_controller = rospy.Publisher(
            '~start_twist_controller', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_twist_limiter = rospy.Publisher(
            '~start_twist_limiter', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_guarded_approach = rospy.Publisher(
            '~start_guarded_approach', std_msgs.msg.String, latch=True, queue_size=1
        )

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~motion_feedback", std_msgs.msg.String, self.motion_feedback_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def motion_feedback_cb(self, msg):
        """
        Obtains the feedback (as event) from the
        arm motion component (e.g. Cartesian motion).

        """
        self.motion_feedback = msg.data

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
            self.event_out.publish('e_stopped')
            self.event = None
            self.motion_feedback = None
            self.started_components = False
            return 'INIT'
        elif self.event == 'e_failure' or self.motion_feedback == 'e_timed_out':
            self.event_out.publish('e_failure')
            self.event = None
            self.motion_feedback = None
            self.started_components = False
            return 'INIT'
        elif self.event == 'e_success':
            self.event_out.publish('e_success')
            self.event = None
            self.motion_feedback = None
            self.started_components = False
            return 'INIT'
        else:
            return 'RUNNING'

    def toggle_components(self, event):
        """
        Starts or stops the necessary components based on the event.

        :param event: The event that determines either to start or stop the components.
        :type event: str

        """
        if self.motion_feedback == 'e_timed_out':
            timed_out = True
        else:
            timed_out = False

        if event == 'e_stop' or event == 'e_failure' or timed_out or event == 'e_success':
            self.start_pose_converter.publish('e_stop')
            self.start_calculator.publish('e_stop')
            self.start_twist_controller.publish('e_stop')
            self.start_twist_limiter.publish('e_stop')
            self.start_guarded_approach.publish('e_stop')
            self.started_components = False

        if event == 'e_start' and not (self.started_components or timed_out):
            self.start_pose_converter.publish('e_start')
            self.start_calculator.publish('e_start')
            self.start_twist_controller.publish('e_start')
            self.start_twist_limiter.publish('e_start')
            self.start_guarded_approach.publish('e_start')
            self.started_components = True


def main():
    rospy.init_node("approach_pose_distance_constrained", anonymous=True)
    approach_pose_distance_constrained = ApproachPoseDistanceConstrained()
    approach_pose_distance_constrained.start()
