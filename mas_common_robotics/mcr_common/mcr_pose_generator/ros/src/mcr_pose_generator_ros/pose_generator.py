#!/usr/bin/env python
"""
This module contains a component that generates a list of poses around a
target pose based on a set of parameters (SphericalSamplerParameters).

"""
#-*- encoding: utf-8 -*-

import math
import numpy
import tf
import rospy
import mcr_pose_generator.transformations as transformations
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg


class PoseGenerator:
    """
    Generates a list of Cartesian poses, based on the specified spherical sample
    parameters and their constraints.

    """
    def __init__(self):
        # params
        self.event = None
        self.target_pose = None
        self.sampling_parameters = None

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # the sampling step for linear variables (in meters)
        self.linear_step = rospy.get_param('~linear_step', 0.01)
        # the sampling step for angular variables (in radians)
        self.angular_step = rospy.get_param('~angular_step', math.pi/12)
        # the maximum amount of samples to be generated
        self.max_samples = rospy.get_param('~max_samples', 50)
        # the configuration matrix of the gripper to be used (a string)
        self.gripper = rospy.get_param('~gripper_config_matrix', None)
        assert self.gripper is not None, "Gripper config matrix needs to be specified"

        # the configuration matrix of the gripper to be used (a real 4x4 matrix)
        self.gripper_config_matrix = rospy.get_param('~' + self.gripper)

        # publishers
        self.poses_list = rospy.Publisher(
            '~poses_list', geometry_msgs.msg.PoseArray, queue_size=1
        )

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            '~target_pose', geometry_msgs.msg.PoseStamped, self.target_pose_cb
        )
        rospy.Subscriber(
            '~sampling_parameters', mcr_manipulation_msgs.msg.SphericalSamplerParameters,
            self.sampling_parameters_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def target_pose_cb(self, msg):
        """
        Obtains the target pose.

        """
        self.target_pose = msg

    def sampling_parameters_cb(self, msg):
        """
        Obtains the spherical sampler parameters.

        """
        self.sampling_parameters = msg

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
        if self.sampling_parameters and self.target_pose:
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'RUNNING'
        elif self.event == 'e_stop':
            self.target_pose = None
            self.sampling_parameters = None
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.target_pose = None
            self.sampling_parameters = None
            return 'INIT'
        else:
            poses_list = self.calculate_poses_list(
                self.target_pose, self.sampling_parameters
            )
            self.poses_list.publish(poses_list)

            return 'RUNNING'

    def calculate_poses_list(self, target_pose, sample_parameters):
        """
        Calculates a list of poses around a target pose based on the spherical sampler parameters.

        :param target_pose: The target pose.
        :type target_pose: geometry_msgs.msg.PoseStamped()

        :param sample_parameters: The parameters to specify a spherical sampling.
        :type sample_parameters: mcr_manipulation_msgs.msg.SphericalSamplerParameters()

        :return: A list of poses around the target pose.
        :rtype: geometry_msgs.msg.PoseArray()

        """
        poses = geometry_msgs.msg.PoseArray()
        poses.header.frame_id = target_pose.header.frame_id
        poses.header.stamp = target_pose.header.stamp

        object_pose = numpy.eye(4)
        object_pose[0, 3] = target_pose.pose.position.x
        object_pose[1, 3] = target_pose.pose.position.y
        object_pose[2, 3] = target_pose.pose.position.z

        gripper_config_matrix = numpy.array(self.gripper_config_matrix)
        transform_matrix = numpy.dot(gripper_config_matrix, object_pose)

        height_offsets = generate_samples(
            sample_parameters.height.minimum, sample_parameters.height.maximum,
            self.linear_step, self.max_samples
        )
        zeniths = generate_samples(
            sample_parameters.zenith.minimum, sample_parameters.zenith.maximum,
            self.angular_step, self.max_samples
        )
        azimuths = generate_samples(
            sample_parameters.azimuth.minimum, sample_parameters.azimuth.maximum,
            self.angular_step, self.max_samples
        )
        wrist_rolls = generate_samples(
            sample_parameters.yaw.minimum, sample_parameters.yaw.maximum,
            self.angular_step, self.max_samples
        )
        radials = generate_samples(
            sample_parameters.radial_distance.minimum,
            sample_parameters.radial_distance.maximum, self.linear_step, self.max_samples
        )

        transforms = [
            transformations.generate_grasp_matrix(
                transform_matrix, height_offset, zenith, azimuth, wrist_roll, radial
            ) for height_offset in height_offsets for zenith in zeniths
            for azimuth in azimuths for wrist_roll in wrist_rolls for radial in radials
        ]

        for transform in transforms:
            pose = matrix_to_pose(transform)
            poses.poses.append(pose)

        return poses


def generate_samples(min_value, max_value, step, max_samples=50):
    """
    Generates an evenly spaced list of samples between the minimum values (min_value)
    and the maximum value (max_value). The minimum and maximum values are both
    inclusive. If the number of samples exceeds the max_samples argument, then only
    max_samples are returned.

    Note: The step might be changed slightly in order to create
          an evenly spaced list.

    :param min_value: The minimum value of the samples.
    :type min_value: float

    :param max_value: The maximum value of the samples.
    :type max_value: float

    :param step: The (approximate) step between samples.
    :type step: float

    :param step: The maximum number of samples allowed.
    :type step: int

    :return: A list of samples.
    :rtype: float[]

    """
    number_of_samples = round((max_value - min_value) / float(step))
    number_of_samples = min([number_of_samples, max_samples - 1])

    return numpy.linspace(min_value, max_value, int(number_of_samples + 1))


def matrix_to_pose(matrix):
    """
    Converts a transformation matrix into a pose, in which the
    orientation is specified by a quaternion.

    :param matrix: The 4x4 transformation matrix.
    :type matrix: numpy.array

    :return: The pose interpretable by ROS.
    :rtype: geometry_msgs.msg.Pose()

    """
    pose = geometry_msgs.msg.Pose()

    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]

    quaternion = tf.transformations.quaternion_from_matrix(matrix)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose


def main():
    rospy.init_node('pose_generator_node', anonymous=True)
    pose_generator = PoseGenerator()
    pose_generator.start()