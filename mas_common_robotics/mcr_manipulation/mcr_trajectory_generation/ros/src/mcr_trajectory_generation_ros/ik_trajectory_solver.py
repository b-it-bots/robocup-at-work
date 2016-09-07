#!/usr/bin/env python
"""
This component computes the inverse kinematics (IK) solution for each pose in a list
of poses.

**Input(s):**
  * `poses`: The poses for which a solution will be calculated.

**Output(s):**
  * `trajectory`: The trajectory, in joint space, specifying only the positions for the
  trajectory.

**Relevant parameter(s):**
  * `max_poses`: Maximum amount of poses to compute a trajectory for.

"""
#-*- encoding: utf-8 -*-

import rospy
import actionlib
import std_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import moveit_msgs.msg
import mcr_trajectory_generation_ros.ik_trajectory_solver_utils as utils
import mcr_manipulation_utils_ros.kinematics as kinematics


class IkTrajectorySolver(object):
    """
    Computes the inverse kinematics (IK) solution for each pose in a list of poses.

    """
    def __init__(self):
        # Params
        self.event = None
        self.poses = None

        # Maximum amount of poses to compute a trajectory for
        self.max_poses = rospy.get_param('~max_poses', 10)

        # Reference frame for the trajectory
        self.reference_frame = rospy.get_param('~reference_frame', None)
        assert self.reference_frame is not None, "Reference frame must be specified."

        # Joint names of the arm
        self.joint_names = rospy.get_param('~joint_names', None)
        assert self.joint_names is not None, "Joint names must be specified."

        # Move group for MoveIt!
        move_group = rospy.get_param('~move_group', None)
        assert move_group is not None, "Move group must be specified."
        client = actionlib.SimpleActionClient(move_group, moveit_msgs.msg.MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        # Name of the group to compute the inverse kinematics
        self.arm = rospy.get_param('~arm', None)
        assert self.arm is not None, "Group to move (e.g. arm) must be specified."

        # Kinematics class to compute the inverse kinematics
        self.kinematics = kinematics.Kinematics(self.arm)

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.trajectory = rospy.Publisher(
            "~trajectory", trajectory_msgs.msg.JointTrajectory, queue_size=1
        )

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~poses", geometry_msgs.msg.PoseArray, self.poses_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def poses_cb(self, msg):
        """
        Obtains the list of poses.

        """
        self.poses = msg

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
            self.event = None
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
            self.event = None
            self.poses = None
            return 'INIT'
        elif self.poses:
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
            self.event = None
            self.poses = None
            return 'INIT'
        else:
            poses = utils.limit_number_of_poses(self.poses, self.max_poses)
            solutions = self.compute_ik_solutions(poses)
            if solutions is not None:
                # todo: check what to do with the time
                trajectory = utils.list_to_joint_trajectory(
                    solutions, self.joint_names, frame_id=self.reference_frame
                )
                self.trajectory.publish(trajectory)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

        self.event = None
        self.poses = None
        return 'IDLE'

    def compute_ik_solutions(self, poses):
        """
        Given a list of poses, it returns a list of joint configurations (as a list)
        that are the solutions for each pose.

        :param poses: A list of poses
        :type poses: geometry_msgs.msg.PoseArray

        :return: A list of joint configurations that reach each pose.
        :rtype: list or None

        """
        # IK solver requires a PoseStamped object
        temp_poses = [geometry_msgs.msg.PoseStamped(pose=pose) for pose in poses.poses]
        rospy.logdebug("Calculating IK solution for {0} poses...".format(len(temp_poses)))

        solutions = [
            self.kinematics.inverse_kinematics(pose)
            for pose in temp_poses
        ]
        if any([solution is None for solution in solutions]):
            rospy.logdebug("IK solution not found for all poses.")
            return None

        rospy.logdebug("IK solution found for all poses.")
        return solutions


def main():
    rospy.init_node('ik_trajectory_solver', anonymous=True)
    ik_trajectory_solver = IkTrajectorySolver()
    ik_trajectory_solver.start()
