#!/usr/bin/env python
"""
This component computes the joint velocities and accelerations given the joint positions
of a trajectory.

**Input(s):**
  * `trajectory_in`: The trajectory, in joint space, specifying only the positions for the
  trajectory.

**Output(s):**
  * `trajectory_out`: The trajectory, in joint space, specifying the positions, velocities
   and accelerations; as well as the duration for each point.

**Relevant parameter(s):**
  * `scaling_factor`: Time scaling factor to control the duration of moving from point 1 to
  point 2 (in seconds).
  * `max_velocity`: Maximum joint velocity allowed.
  * `max_acceleration`: Maximum joint acceleration allowed.

"""
#-*- encoding: utf-8 -*-

import copy
import itertools
import rospy
import mcr_trajectory_generation_ros.trajectory_generator_utils as utils
import std_msgs.msg
import trajectory_msgs.msg


class TrajectoryGenerator(object):
    """
    Computes the joint velocities and accelerations given the joint positions
    of a trajectory.

    """
    def __init__(self):
        # Params
        self.event = None
        self.trajectory_in = None

        # Time scaling factor to control the duration of moving from point 1 to
        # point 2 (in seconds).
        self.scaling_factor = rospy.get_param('scaling_factor', 1.0)

        # Maximum joint velocity allowed.
        self.max_velocity = rospy.get_param('max_velocity', None)

        # Maximum joint acceleration allowed.
        self.max_acceleration = rospy.get_param('max_acceleration', None)

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.trajectory_out = rospy.Publisher(
            "~trajectory_out", trajectory_msgs.msg.JointTrajectory, queue_size=1
        )

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
            trajectory = self.compute_trajectory(self.trajectory_in)
            if trajectory is not None:
                self.trajectory_out.publish(trajectory)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

        self.reset_inputs()
        return 'IDLE'

    def reset_inputs(self):
        """
        Resets the component's inputs.

        """
        self.event = None
        self.trajectory_in = None

    def compute_trajectory(self, trajectory_in):
        """
        Computes the joint velocities and accelerations given the joint positions
        of a trajectory, as well as the duration from point to point.

        :param trajectory_in: A trajectory with the joint position information.
        :type trajectory_in: trajectory_msgs.msg.JointTrajectory

        :return: A trajectory with joint positions, velocities and accelerations; as
        well as the duration from point to point.
        :rtype: trajectory_msgs.msg.JointTrajectory

        """
        trajectory_out = copy.deepcopy(trajectory_in)
        positions = [point.positions for point in trajectory_out.points]
        zero_vector = [0 for _ in positions[0]]

        # Calculate time_from_start for each point
        read_ahead = iter(positions)
        next(read_ahead)
        durations = [
            utils.calculate_maximum_duration(p1, p2, self.scaling_factor)
            for p1, p2 in itertools.izip(positions, read_ahead)
        ]
        # Duration for the first point is assumed to be zero
        durations.insert(0, 0.0)

        # Calculate velocity vector for each point
        read_ahead = iter(positions)
        next(read_ahead)
        velocities = [
            utils.calculate_discrete_difference(p1, p2, self.max_velocity)
            for p1, p2 in itertools.izip(positions, read_ahead)
        ]
        # Ensure the last point has zero velocity
        velocities.append(zero_vector)

        # Calculate acceleration vector for each point
        read_ahead = iter(velocities)
        next(read_ahead)
        accelerations = [
            utils.calculate_discrete_difference(v1, v2, self.max_velocity)
            for v1, v2 in itertools.izip(velocities, read_ahead)
        ]
        # zero_vector the last point has zero acceleration
        accelerations.append(zero_vector)

        time = 0
        for point, duration, vel, acc in zip(
                trajectory_out.points, durations, velocities, accelerations
        ):
            time += duration
            point.time_from_start = rospy.Duration(time)
            point.velocities = vel
            point.accelerations = acc

        return trajectory_out


def main():
    rospy.init_node('trajectory_generator', anonymous=True)
    trajectory_generator = TrajectoryGenerator()
    trajectory_generator.start()
