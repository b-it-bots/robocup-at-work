#!/usr/bin/env python
"""
Kinematics module.

"""
#-*- encoding: utf-8 -*-

import rospy
import moveit_msgs.msg
import moveit_msgs.srv
import moveit_commander
import extractors


class Kinematics:
    def __init__(self, group_name):
        self.group_name = group_name
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.commander = moveit_commander.RobotCommander()
        self.state = moveit_commander.RobotState()
        self.joint_names = self.commander.get_joint_names(self.group_name)

        # service clients
        rospy.loginfo("Waiting for 'compute_ik' service")
        rospy.wait_for_service('/compute_ik')
        self.ik_client = rospy.ServiceProxy('/compute_ik',
                                            moveit_msgs.srv.GetPositionIK)
        rospy.loginfo("Found service 'compute_ik'")

    def inverse_kinematics(self, goal_pose, configuration=None, timeout=0.5, attempts=1):
        """
        Calls the IK solver to calculate the joint configuration to reach the
        goal pose. The configuration parameter is the start position of the
        joints. If no configuration is provided, the current joint values are
        used as the initial joint configuration.

        :param goal_pose: The pose for which the inverse kinematics is solved
        :type goal_pose: geometry_msgs.msg.PoseStamped

        :param configuration: The initial manipulator's configuration
        :type configuration: Float[]

        :param timeout: Time allowed for the IK solver to find a solution.
        :type timeout: Float

        :param attempts: Number of  attempts the IK solver tries to find a solution.
        :type attempts: Int

        :return: The joint configuration that reaches the requested pose
        :rtype: Float[] or None

        """
        if not configuration:
            configuration = self.group.get_current_joint_values()

        if len(self.joint_names) != len(configuration):
            return None

        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.timeout = rospy.Duration(timeout)
        req.ik_request.attempts = attempts
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = configuration
        req.ik_request.pose_stamped = goal_pose
        try:
            resp = self.ik_client(req)
        except rospy.ServiceException, e:
            rospy.logerr('Service did not process request: %s', str(e))
            return None

        if resp.error_code.val == resp.error_code.SUCCESS:
            return extractors.extract_positions(
                resp.solution.joint_state, self.joint_names
            )
        else:
            return None
