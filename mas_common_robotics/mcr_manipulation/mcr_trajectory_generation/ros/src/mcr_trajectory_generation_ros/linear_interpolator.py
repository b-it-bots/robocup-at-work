#!/usr/bin/env python
"""
This component generates a path (in task space) to reach a goal pose from a
start pose based on linear interpolation. It is assumed that the start and goal
poses have the same reference frame.

**Input(s):**
  * `start_pose`: The pose where the manipulator's end effector will start the
  trajectory.
  * `goal_pose`: The pose where the manipulator's end effector will end the trajectory.

**Output(s):**
  * `path`: The path connecting the start and goal poses on a straight line.

**Relevant parameter(s):**
  * `max_poses`: Number of poses to generate between the start and goal pose
  (including those two), e.g. specifying 5 'max_poses' will output the start pose
  as the first pose, then three poses equally separated, and finally the goal pose
  as the fifth pose.

"""
#-*- encoding: utf-8 -*-

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_trajectory_generation_ros.linear_interpolator_utils as utils


class LinearInterpolator(object):
    """
This component generates a path (in task space) to reach a goal pose from a
start pose based on linear interpolation.

    """
    def __init__(self):
        # Params
        self.event = None
        self.start_pose = None
        self.goal_pose = None

        # Number of poses to generate between the start and goal pose (including those
        # two)
        self.max_poses = rospy.get_param('~max_poses', 5)

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.computed_path = rospy.Publisher("~path", geometry_msgs.msg.PoseArray, queue_size=1)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~start_pose", geometry_msgs.msg.PoseStamped, self.start_pose_cb
        )
        rospy.Subscriber(
            "~goal_pose", geometry_msgs.msg.PoseStamped, self.goal_pose_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def start_pose_cb(self, msg):
        """
        Obtains the start configuration.

        """
        self.start_pose = msg

    def goal_pose_cb(self, msg):
        """
        Obtains the target configuration.

        """
        self.goal_pose = msg

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
        elif self.start_pose and self.goal_pose:
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
            path = self.calculate_path(self.start_pose, self.goal_pose, self.max_poses)
            if path is not None:
                self.computed_path.publish(path)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')
                self.reset_inputs()
                return 'INIT'

        self.reset_inputs()
        return 'IDLE'

    def reset_inputs(self):
        """
        Resets the component's inputs.

        """
        self.event = None
        self.start_pose = None
        self.goal_pose = None

    def calculate_path(self, start_pose, goal_pose, max_poses):
        """
        Calculates a path, with 'n' max_poses, based on a linear interpolation
        between the start and goal poses. This path includes the start pose as
        the first pose and the goal pose as the last pose.
        Assumption: The generated poses will have the same orientation as the start pose.

        Note: The start and goal poses must be specified in the same frame of reference.

        :param start_pose: The first pose of the path.
        :type start_pose: geometry_msgs.msg.PoseStamped

        :param goal_pose: The last pose of the path.
        :type goal_pose: geometry_msgs.msg.PoseStamped

        :param max_poses: Number of poses to generate between the start and goal pose
        (including those two), e.g. specifying 5 'max_poses' will output the start
        pose as the first pose, then three poses equally separated, and finally the
        goal pose as the fifth pose.
        :type max_poses: int

        :return: The path as a PoseArray message.
        :rtype: geometry_msgs.msg.PoseArray or None

        """
        assert (start_pose.header.frame_id == goal_pose.header.frame_id),\
            "The frame id of the start and goal pose must be the same.\n'{0}' != '{1}'" \
            "".format(self.start_pose.header.frame_id, self.goal_pose.header.frame_id)

        path = geometry_msgs.msg.PoseArray()
        path.header.frame_id = self.start_pose.header.frame_id

        # Start pose should be the first pose in the path
        path.poses.append(self.start_pose.pose)

        interpolated_poses = utils.interpolate_poses(
            self.start_pose, self.goal_pose, max_poses - 2
        )
        path.poses.extend(interpolated_poses)

        # Goal pose should be the last pose in the path
        path.poses.append(self.goal_pose.pose)

        path.header.stamp = rospy.Time.now()
        return path


def main():
    rospy.init_node('linear_interpolator', anonymous=True)
    linear_interpolator = LinearInterpolator()
    linear_interpolator.start()
