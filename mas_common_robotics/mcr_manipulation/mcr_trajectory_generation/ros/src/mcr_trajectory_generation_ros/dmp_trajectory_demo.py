#!/usr/bin/env python
"""
A demo to generate a trajectory using DMPs and subsequently moving the arm to follow
such trajectory.

This component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

**Input(s):**
  * `start_pose`: The pose where the manipulator's end effector will start the
  trajectory.
  * `goal_pose`: The pose where the manipulator's end effector will end the trajectory.

**Output(s):**
  * `trajectory`: The trajectory, in joint space, connecting the start and goal poses as
  a goal.

"""
#-*- encoding: utf-8 -*-

import rospy
import std_msgs.msg


class DmpTrajectoryDemo(object):
    """
    Coordinates components that compute and execute a joint trajectory based on a
    start and goal pose.

    """
    def __init__(self):
        # Params
        self.started_components = False
        self.event = None
        self.dmp_generator_status = None
        self.ik_solver_status = None
        self.trajectory_generator_status = None

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.start_dmp_generator = rospy.Publisher(
            '~start_dmp_generator', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_ik_solver = rospy.Publisher(
            '~start_ik_solver', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_trajectory_generator = rospy.Publisher(
            '~start_trajectory_generator', std_msgs.msg.String, latch=True, queue_size=1
        )

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~dmp_generator_status", std_msgs.msg.String, self.dmp_generator_status_cb
        )
        rospy.Subscriber(
            "~ik_solver_status", std_msgs.msg.String, self.ik_solver_status_cb
        )
        rospy.Subscriber(
            "~trajectory_generator_status", std_msgs.msg.String,
            self.trajectory_generator_status_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def dmp_generator_status_cb(self, msg):
        """
        Obtains the status of the trajectory generator (as an event).

        """
        self.dmp_generator_status = msg.data

    def ik_solver_status_cb(self, msg):
        """
        Obtains the status of the ik solver (as an event).

        """
        self.ik_solver_status = msg.data

    def trajectory_generator_status_cb(self, msg):
        """
        Obtains the status of the joint trajectory generator (as an event).

        """
        self.trajectory_generator_status = msg.data

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
        if (self.dmp_generator_status == 'e_failure'
            or self.ik_solver_status == 'e_failure'
                or self.trajectory_generator_status == 'e_failure'):
            status = 'e_failure'
            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        if self.trajectory_generator_status == 'e_success':
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
            self.start_dmp_generator.publish('e_stop')
            self.start_ik_solver.publish('e_stop')
            self.start_trajectory_generator.publish('e_stop')
            self.started_components = False

        if event == 'e_start' and not self.started_components:
            self.start_dmp_generator.publish('e_start')
            self.start_ik_solver.publish('e_start')
            self.start_trajectory_generator.publish('e_start')
            self.started_components = True

    def reset_component_data(self, result):
        """
        Clears the data of the component.

        :param result: The result of the component, e.g. stopped, failure, success.
        :type result: str

        """
        self.toggle_components(result)
        self.event = None
        self.dmp_generator_status = None
        self.ik_solver_status = None
        self.trajectory_generator_status = None
        self.started_components = False


def main():
    rospy.init_node("dmp_trajectory_demo", anonymous=True)
    dmp_trajectory_demo = DmpTrajectoryDemo()
    dmp_trajectory_demo.start()
