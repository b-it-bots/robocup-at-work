#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This component moves the mobile base in Cartesian space until a pose is reached.

It uses the following nodes:
  * (mcr_common_converters) transform_to_pose_converter.
  * (mcr_manipulation_measurers) component_wise_pose_error_calculator.
  * (mcr_geometric_relation_monitors) component_wise_pose_error_monitor.
  * (mcr_twist_controller) twist_controller.
  * (mcr_twist_limiter) twist_limiter.
  * (mcr_twist_synchronizer) twist_synchronizer.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_monitoring_msgs.msg


class DirectBaseControllerCoordinator(object):
    """
    Components that move the base until a pose is reached.

    """
    def __init__(self):
        # params
        self.started_components = False
        self.event = None
        self.pose_monitor_feedback = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100.0))

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.pose_converter_event_in = rospy.Publisher(
            '~pose_converter_event_in', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.pose_error_calculator_event_in = rospy.Publisher(
            '~pose_error_calculator_event_in', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.pose_error_monitor_event_in = rospy.Publisher(
            '~pose_error_monitor_event_in', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.twist_controller_event_in = rospy.Publisher(
            '~twist_controller_event_in', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.twist_limiter_event_in = rospy.Publisher(
            '~twist_limiter_event_in', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.twist_synchronizer_event_in = rospy.Publisher(
            '~twist_synchronizer_event_in', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.base_twist = rospy.Publisher('~twist', geometry_msgs.msg.Twist, queue_size=1)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~pose_monitor_feedback", mcr_monitoring_msgs.msg.ComponentWisePoseErrorMonitorFeedback,
                         self.pose_monitor_feedback_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def pose_monitor_feedback_cb(self, msg):
        """
        Obtains the feedback from the pose error monitor component

        """
        self.pose_monitor_feedback = msg

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
            self.send_event_to_components("start")
            self.event = None
            return 'RUNNING'
        else:
            return 'INIT'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """

        if self.event == 'e_stop':
            self.send_event_to_components("stop")
            self.event_out.publish('e_stopped')
            self.event = None
            self.pose_monitor_feedback = None
            self.started_components = False

            self.publish_zero_velocities()

            return 'INIT'

        if self.pose_monitor_feedback is not None:
            if self.pose_monitor_feedback.is_linear_x_within_tolerance and \
                    self.pose_monitor_feedback.is_linear_y_within_tolerance and \
                    self.pose_monitor_feedback.is_angular_z_within_tolerance:
                self.send_event_to_components("stop")
                self.event_out.publish('e_success')
                self.event = None
                self.pose_monitor_feedback = None
                self.started_components = False

                self.publish_zero_velocities()

                return 'INIT'

        return 'RUNNING'

    def publish_zero_velocities(self):
        zero_twist = geometry_msgs.msg.Twist()

        self.base_twist.publish(zero_twist)

    def send_event_to_components(self, event):
        """
        Starts or stops the necessary components

        :param event: The event that determines either to start or stop the components.
        :type event: str

        """
        if event == 'start' and not (self.started_components):
            self.pose_converter_event_in.publish('e_start')
            self.pose_error_calculator_event_in.publish('e_start')
            self.pose_error_monitor_event_in.publish('e_start')
            self.twist_controller_event_in.publish('e_start')
            self.twist_limiter_event_in.publish('e_start')
            self.twist_synchronizer_event_in.publish('e_start')

            self.started_components = True

        elif event == 'stop':
            self.pose_converter_event_in.publish('e_stop')
            self.pose_error_calculator_event_in.publish('e_stop')
            self.pose_error_monitor_event_in.publish('e_stop')
            self.twist_controller_event_in.publish('e_stop')
            self.twist_limiter_event_in.publish('e_stop')
            self.twist_synchronizer_event_in.publish('e_stop')

            self.started_components = False


def main():
    rospy.init_node("direct_base_controller", anonymous=True)
    direct_base_controller_coordinator = DirectBaseControllerCoordinator()
    direct_base_controller_coordinator.start()
