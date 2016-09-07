#!/usr/bin/env python
"""
This component moves a manipulator in Cartesian space until a pose is reached.
It uses two nodes:
  * (mcr_arm_motions) cartesian_motion_node and
  * (mcr_geometric_relation_monitors) component_wise_pose_error_monitor.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for both components and starts/stops them accordingly.

"""
#-*- encoding: utf-8 -*-

import numpy
import rospy
import std_msgs.msg
import mcr_monitoring_msgs.msg


class DistanceConstrained(object):
    """
    Components that move the arm until a pose is reached.

    """
    def __init__(self):
        # params
        self.started_components = False
        self.in_tolerance = False
        self.event = None
        self.monitor_feedback = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # a boolean list containing which elements of a component-wise pose
        # error should be checked if they are within tolerance
        self.desired_tolerances = rospy.get_param('~desired_tolerances', None)
        assert self.desired_tolerances is not None, "Desired tolerances must not be empty."

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.start_monitor = rospy.Publisher(
            '~start_monitor', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_cartesian_motion = rospy.Publisher(
            '~start_cartesian_motion', std_msgs.msg.String, latch=True, queue_size=1
        )

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~monitor_feedback",
            mcr_monitoring_msgs.msg.ComponentWisePoseErrorMonitorFeedback,
            self.monitor_feedback_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def monitor_feedback_cb(self, msg):
        """
        Obtains the feedback of the monitor.

        """
        self.monitor_feedback = msg

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
        if self.event == 'e_stop':
            self.toggle_components()
            self.monitor_feedback = None
            self.event = None
            self.in_tolerance = False
            return 'INIT'
        else:
            self.toggle_components()
            if self.monitor_feedback:
                self.check_tolerance(self.desired_tolerances)
            return 'RUNNING'

    def toggle_components(self):
        """
        Starts or stops (depending on the events) the necessary components.

        """
        if self.event == 'e_stop':
            self.start_monitor.publish('e_stop')
            self.start_cartesian_motion.publish('e_stop')
            self.started_components = False
            if not self.in_tolerance:
                self.event_out.publish('e_failure')

        if self.in_tolerance:
            self.start_monitor.publish('e_stop')
            self.start_cartesian_motion.publish('e_stop')
            self.event_out.publish('e_success')
            self.event = None
            self.monitor_feedback = None
            self.started_components = False

        if self.event == 'e_start' and not self.started_components:
            self.start_monitor.publish('e_start')
            self.start_cartesian_motion.publish('e_start')
            self.started_components = True

    def check_tolerance(self, desired_tolerances):
        """
        Verifies that the desired elements of a component-wise error
        are within tolerance.

        """
        monitor_feedback_attributes = [
            aa for aa in dir(self.monitor_feedback) if "tolerance" in aa
        ]

        required_tolerances = [
            string_in_list_element(tol, monitor_feedback_attributes)
            for tol in desired_tolerances
        ]

        tolerances_cleared = [
            getattr(self.monitor_feedback, str(rr), None) for rr in required_tolerances
        ]

        self.in_tolerance = numpy.array(tolerances_cleared).all()


def string_in_list_element(string, list_of_strings):
    """
    Returns an element of list_of_strings, if string is found in any of them.
    Returns None, if string is not found.

    :param string: The string to look for in the list of strings.
    :type string: str

    :param list_of_strings: A list of strings.
    :type list_of_strings: list

    :return: The (first found) element in the list of strings containing the
        string or None, if the string is not found in the list of strings.
    :rtype: str or None

    """
    found_strings = [ss for ss in list_of_strings if string in ss]
    if found_strings:
        return found_strings[0]
    else:
        return None


def main():
    rospy.init_node("distance_constrained", anonymous=True)
    distance_constrained = DistanceConstrained()
    distance_constrained.start()
