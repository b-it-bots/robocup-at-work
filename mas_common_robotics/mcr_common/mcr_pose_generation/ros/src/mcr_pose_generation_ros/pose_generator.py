#!/usr/bin/env python
"""
This component generates a list of poses around a target pose based on a set of
parameters (SphericalSamplerParameters).

**Input(s):**
  * `target_pose`: The target pose from which to create a set of poses around that
  object.
  * `sampling_parameters`: A message specifying the parameters, and constraints,
  of the pose to be sampled around an object, if any.

**Output(s):**
  * `poses_list`: The list of poses around the target pose as defined by the
  `sampling_parameters`.

**Parameter(s):**
  * `linear_step`: Sampling step for linear variables (in meters).
  * `angular_step`: Sampling step for angular variables (in degrees).
  * `max_poses`: Maximum amount of samples to be generated (int).
  * `gripper`: Configuration matrix of the gripper to be used (as a string).
  * `loop_rate`: Node cycle rate (in hz).

"""
#-*- encoding: utf-8 -*-

import math
import tf
import rospy
import mcr_pose_generation.transformations as transformations
import mcr_pose_generation_ros.pose_generator_utils as utils
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg


class PoseGenerator:
    """
    Generates a list of Cartesian poses, based on the specified spherical sample
    parameters and their constraints.

    """
    def __init__(self):
        # Params
        self.event = None
        self.target_pose = None
        self.sampling_parameters = None

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))
        # Sampling step for linear variables (in meters)
        self.linear_step = rospy.get_param('~linear_step', 0.01)
        # Sampling step for angular variables (in degrees)
        self.angular_step = math.radians(rospy.get_param('~angular_step', 15))
        # Maximum amount of samples to be generated (int)
        self.max_samples = rospy.get_param('~max_samples', 50)
        # Configuration matrix of the gripper to be used (as a string)
        self.gripper = rospy.get_param('~gripper_config_matrix', None)
        assert self.gripper is not None, "Gripper config matrix must be specified."

        # Configuration matrix of the gripper to be used (a real 4x4 matrix)
        self.gripper_config_matrix = rospy.get_param('~' + self.gripper)

        # Publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=1)
        self.poses_list = rospy.Publisher(
            '~poses_list', geometry_msgs.msg.PoseArray, queue_size=1
        )

        # Subscribers
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
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        elif self.sampling_parameters and self.target_pose:
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
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            poses_list = self.calculate_poses_list(
                self.target_pose, self.sampling_parameters
            )
            if poses_list:
                self.poses_list.publish(poses_list)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

            self.reset_component_data()
            return 'IDLE'

    def calculate_poses_list(self, target_pose, sample_parameters, number_of_fields=5):
        """
        Calculates a list of poses around a target pose based on the spherical sampler parameters.

        :param target_pose: The target pose.
        :type target_pose: geometry_msgs.msg.PoseStamped()

        :param sample_parameters: The parameters to specify a spherical sampling.
        :type sample_parameters: mcr_manipulation_msgs.msg.SphericalSamplerParameters()

        :param number_of_fields: The number of fields the SphericalSamplerParameters
            message has.
        :type number_of_fields: int

        :return: A list of poses around the target pose.
        :rtype: geometry_msgs.msg.PoseArray()

        """
        poses = geometry_msgs.msg.PoseArray()
        poses.header.frame_id = target_pose.header.frame_id
        poses.header.stamp = target_pose.header.stamp

        object_matrix = tf.transformations.quaternion_matrix([
            target_pose.pose.orientation.x, target_pose.pose.orientation.y,
            target_pose.pose.orientation.z, target_pose.pose.orientation.w
        ])
        object_matrix[0, 3] = target_pose.pose.position.x
        object_matrix[1, 3] = target_pose.pose.position.y
        object_matrix[2, 3] = target_pose.pose.position.z

        height_offsets = utils.generate_samples(
            sample_parameters.height.minimum, sample_parameters.height.maximum,
            self.linear_step, (self.max_samples / number_of_fields)
        )
        zeniths = utils.generate_samples(
            sample_parameters.zenith.minimum, sample_parameters.zenith.maximum,
            self.angular_step, (self.max_samples / number_of_fields)
        )
        azimuths = utils.generate_samples(
            sample_parameters.azimuth.minimum, sample_parameters.azimuth.maximum,
            self.angular_step, (self.max_samples / number_of_fields)
        )
        wrist_rolls = utils.generate_samples(
            sample_parameters.yaw.minimum, sample_parameters.yaw.maximum,
            self.angular_step, (self.max_samples / number_of_fields)
        )
        radials = utils.generate_samples(
            sample_parameters.radial_distance.minimum,
            sample_parameters.radial_distance.maximum, self.linear_step,
            (self.max_samples / number_of_fields)
        )

        transforms = [
            transformations.generate_grasp_matrix(
                object_matrix, self.gripper_config_matrix, height_offset,
                zenith, azimuth, wrist_roll, radial
            ) for height_offset in height_offsets for zenith in zeniths
            for azimuth in azimuths for wrist_roll in wrist_rolls for radial in radials
        ]

        for transform in transforms:
            pose = utils.matrix_to_pose(transform)
            poses.poses.append(pose)

        return poses

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.target_pose = None
        self.sampling_parameters = None


def main():
    rospy.init_node('pose_generator_node', anonymous=True)
    pose_generator = PoseGenerator()
    pose_generator.start()
