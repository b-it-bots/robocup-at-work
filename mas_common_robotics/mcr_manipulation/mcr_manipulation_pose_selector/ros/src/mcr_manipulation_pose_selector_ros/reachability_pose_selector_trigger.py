#!/usr/bin/env python
"""
This module contains a component that publishes arm joint configuration 
for desired Pose or list of poses.

"""
#-*- encoding: utf-8 -*-

import rospy
import actionlib
import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import brics_actuator.msg
import moveit_commander
import mcr_manipulation_utils_ros.kinematics as kinematics
import mcr_manipulation_pose_selector_ros.reachability_pose_selector_utils as pose_selector_utils


class PoseSelector(object):
    """
    Publishes a Joint Configuration based desired pose.

    """
    def __init__(self):
        
        # params
        self.event = None
        self.goal_pose = None
        self.goal_pose_array = None
        
        # wait for MoveIt!
        move_group = rospy.get_param('~move_group', None)
        assert move_group is not None, "Move group must be specified."
        client = actionlib.SimpleActionClient(move_group, moveit_msgs.msg.MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        # name of the group to compute the inverse kinematics
        self.arm = rospy.get_param('~arm', None)
        assert self.arm is not None, "Group to move (e.g. arm) must be specified."

        group = moveit_commander.MoveGroupCommander(self.arm)
        # joints to compute the inverse kinematics
        self.joint_uris = group.get_joints()

        # units of the joint position values
        self.units = rospy.get_param('~units', 'rad')

        # linear offset for the X, Y and Z axis.
        self.linear_offset = rospy.get_param('~linear_offset', None)

        # kinematics class to compute the inverse kinematics
        self.kinematics = kinematics.Kinematics(self.arm)

        # Time allowed for the IK solver to find a solution (in seconds).
        self.ik_timeout = rospy.get_param('~ik_timeout', 0.5)

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.selected_pose = rospy.Publisher(
            "~selected_pose", geometry_msgs.msg.PoseStamped, queue_size=1
        )
        self.joint_configuration = rospy.Publisher(
            "~joint_configuration", brics_actuator.msg.JointPositions, queue_size=1
        )

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~goal_pose", geometry_msgs.msg.PoseStamped, self.goal_pose_cb)
        rospy.Subscriber(
            "~goal_pose_array", geometry_msgs.msg.PoseArray, self.goal_pose_array_cb
        )

    def goal_pose_cb(self, msg):
        """
        Obtains the goal pose.

        """
        self.goal_pose = msg

    def goal_pose_array_cb(self, msg):
        """
        Obtains an array of goal poses.

        """
        self.goal_pose_array = msg
        
    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

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
            self.event = None
            self.goal_pose = None
            self.goal_pose_array = None
            return 'INIT'
        elif self.goal_pose or self.goal_pose_array:
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
            self.event = None
            self.goal_pose = None
            self.goal_pose_array = None
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            poses = self.group_goal_poses()
            solution = self.select_reachable_pose(poses, self.linear_offset)
            self.publish_result(solution)

            self.event = None
            self.goal_pose = None
            self.goal_pose_array = None
            return 'INIT'

    def group_goal_poses(self):
        """
        Returns a list of PoseStamped objects based on the input to the node.

        :return: A list of the goal poses.
        :rtype: list or None

        """
        poses = None
        if self.goal_pose_array:
            poses = pose_selector_utils.pose_array_to_list(self.goal_pose_array)
        if self.goal_pose:
            if poses is None:
                poses = [self.goal_pose]
            else:
                poses.append(self.goal_pose)
        return poses

    def select_reachable_pose(self, poses, offset):
        """
        Given a list of poses, it returns the first pose that returns a
        solution and the joint configuration for that solution.

        :param poses: A list of geometry_msgs.msg.PoseStamped objects.
        :type poses: list

        :param offset: A linear offset for the X, Y and Z axis.
        :type offset: list

        :return: The pose for which a solution exists and the joint configuration
            that reaches that pose (of the requested poses).
        :rtype: (geometry_msgs.msg.PoseStamped, list) or None

        """
        for ii, pose in enumerate(poses):
            rospy.logdebug("IK solver attempt number: {0}".format(ii))
            if offset:
                solution = self.kinematics.inverse_kinematics(
                    pose_selector_utils.add_linear_offset_to_pose(pose, offset),
                    timeout=self.ik_timeout
                )
            else:
                solution = self.kinematics.inverse_kinematics(
                    pose, timeout=self.ik_timeout
                )
            if solution:
                return pose, solution
        print("No solution found")
        return None

    def publish_result(self, solution):
        """
        Publishes an 'e_success' event and the solution if one exists,
        otherwise only an 'e_failure' event is published.

        :param solution: A pose with the joint configuration that reaches that pose,
            or None.
        :type solution: (geometry_msgs.msg.PoseStamped, list) or None

        """
        if solution is not None:
            pose = solution[0]
            joint_values = solution[1]
            if pose.header.stamp:
                configuration = pose_selector_utils.list_to_brics_joints(
                    joint_values, self.joint_uris, time_stamp=pose.header.stamp,
                    unit=self.units
                )
            else:
                configuration = pose_selector_utils.list_to_brics_joints(
                    joint_values, self.joint_uris, unit=self.units
                )
            self.selected_pose.publish(pose)
            self.joint_configuration.publish(configuration)
            self.event_out.publish('e_success')
        else:
            self.event_out.publish('e_failure')


def main():
    rospy.init_node("reachability_pose_selector", anonymous=True)
    pose_selector = PoseSelector()
    pose_selector.start()
