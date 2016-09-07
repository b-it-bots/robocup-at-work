#!/usr/bin/env python
"""
This component uses the `planned_motion` component and it provides feedback
whether the desired configuration is reached.

It uses the following nodes:
  * (mcr_arm_motions) `planned_motion`.
  * (mcr_topic_tools) `brics_joints_to_joint_states`.
  * (mcr_manipulation_monitors) `joint_position_monitor`.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

**Input(s):**
  * `target_configuration`: The joint configuration to which the manipulator will
  be moved.

**Parameter(s):**
  * `joint_position_monitor`
    * `target_joint_names`: Names of the joints to be monitored.
    * `epsilon`: Tolerance, as joint position value, for each joint.

"""
#-*- encoding: utf-8 -*-

import rospy
import std_msgs.msg


class PlannedMotionWithFeedback(object):
    """
    Coordinates components to move a robot manipulator to a desired joint
    configuration and provides feedback of its result.

    """
    def __init__(self):
        # Params
        self.started_components = False
        self.event = None
        self.monitor_status = None

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)
        self.start_planned_motion = rospy.Publisher(
            '~start_planned_motion', std_msgs.msg.String, latch=True
        )
        self.start_monitor = rospy.Publisher(
            '~start_monitor', std_msgs.msg.String, latch=True
        )

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~monitor_status", std_msgs.msg.String, self.monitor_status_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def monitor_status_cb(self, msg):
        """
        Obtains the status of the joint position monitor (as an event).

        """
        self.monitor_status = msg.data

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
            self.reset_component_data()
            return 'INIT'
        if self.monitor_status == 'e_fail':
            self.event_out.publish('e_failure')
            self.reset_component_data()
            return 'INIT'
        if self.monitor_status == 'e_done':
            self.event_out.publish('e_success')
            self.reset_component_data()
            return 'INIT'
        else:
            return 'RUNNING'

    def toggle_components(self, event):
        """
        Starts or stops the necessary components based on the event.

        :param event: The event that determines either to start or stop the components.
        :type event: str or None

        """
        if event == 'e_stop' or event == 'e_failure' or event == 'e_done':
            self.start_planned_motion.publish('e_stop')
            self.start_monitor.publish('e_stop')
            self.started_components = False

        if event == 'e_start' and not self.started_components:
            self.start_planned_motion.publish('e_start')
            self.start_monitor.publish('e_start')
            self.started_components = True

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.monitor_status = None
        self.started_components = False


def main():
    rospy.init_node("planned_motion_with_feedback", anonymous=True)
    planned_motion_with_feedback = PlannedMotionWithFeedback()
    planned_motion_with_feedback.start()
