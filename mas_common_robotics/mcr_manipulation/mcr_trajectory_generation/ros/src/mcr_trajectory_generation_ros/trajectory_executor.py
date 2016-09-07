#!/usr/bin/env python
"""
This component executes a specified trajectory.

**Input(s):**
  * `trajectory_in`: The trajectory, in joint space, specifying the positions, velocities
   and accelerations; as well as the duration for each point.

**Output(s):**
  * `trajectory_out`: The specified trajectory as a trajectory goal.

**Relevant parameter(s):**
  * `trajectory_controller`: Trajectory controller to be used to execute the trajectory

"""
#-*- encoding: utf-8 -*-

import copy
import rospy
import actionlib
import std_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg


class TrajectoryExecutor(object):
    """
    Executes the specified trajectory.

    """
    def __init__(self):
        # Params
        self.event = None
        self.trajectory_in = None

        # Trajectory controller to be used to execute the trajectory
        trajectory_controller = rospy.get_param('~trajectory_controller', None)
        assert trajectory_controller is not None,\
            "'trajectory_controller' group must be specified."
        self.client = actionlib.SimpleActionClient(
            trajectory_controller, control_msgs.msg.FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for '{0}' server".format(trajectory_controller))
        self.client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(trajectory_controller))

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~trajectory_in", trajectory_msgs.msg.JointTrajectory, self.trajectory_in_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def trajectory_in_cb(self, msg):
        """
        Obtains the trajectory.

        """
        self.trajectory_in = msg

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
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
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event_out.publish('e_stopped')
            self.reset_inputs()
            return 'INIT'
        elif self.trajectory_in:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event_out.publish('e_stopped')
            self.reset_inputs()
            return 'INIT'
        else:
            goal = control_msgs.msg.FollowJointTrajectoryGoal()
            goal.trajectory = copy.deepcopy(self.trajectory_in)

            self.client.send_goal(goal)
            self.client.wait_for_result()
            self.event_out.publish('e_success')

        self.reset_inputs()
        return 'INIT'

    def reset_inputs(self):
        """
        Resets the component's inputs.

        """
        self.event = None
        self.trajectory_in = None


def main():
    rospy.init_node('trajectory_executor', anonymous=True)
    trajectory_executor = TrajectoryExecutor()
    trajectory_executor.start()
