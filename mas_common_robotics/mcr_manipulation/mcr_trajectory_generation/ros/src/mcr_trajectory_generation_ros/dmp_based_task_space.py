#!/usr/bin/env python
"""
This component generates a trajectory (in task space) to reach a goal pose from a
start pose based on an example trajectory (primitive_motion). It uses
Dynamic Movement Primitives (DMPs) allowing arbitrary start/target configurations [1].

[1] http://wiki.ros.org/dmp


**Input(s):**
  * `start_pose`: The pose where the manipulator's end effector will start the
  trajectory.
  * `goal_pose`: The pose where the manipulator's end effector will end the trajectory.

**Output(s):**
  * `trajectory`: The trajectory connecting the start and goal poses, based on the
  example trajectory.

**Relevant parameter(s):**
  * `primitive_motion`: Example trajectory showing a motion (e.g. grasp, place).
  This can be saved in a config file (see example config file).
  * `reference_frame`: Frame of reference for the generated trajectory.
  * `duration`: Desired duration of the trajectory (scaled to the duration of the
  demonstrated trajectory, e.g. a value of 2 will generate a plan twice as long as
  the demonstrated one).

"""
#-*- encoding: utf-8 -*-

import numpy
import rospy
import dmp
import std_msgs.msg
import geometry_msgs.msg
import dmp.msg
import dmp.srv
import dynamic_reconfigure.server
import mcr_trajectory_generation.cfg.TaskSpaceParamsConfig as TaskSpaceParams


class DmpTaskSpace(object):
    """
    This component generates a trajectory (in task space) to reach a goal pose
    from a start pose based on an example trajectory (primitive_motion)
    using Dynamic Movement Primitives (DMPs).

    """
    def __init__(self):
        # Params
        self.event = None
        self.start_pose = None
        self.goal_pose = None
        # Learn from Demonstration response
        self.lfd_response = None
        # Flag to check if a DMP has been learned from a demonstration
        self.is_dmp_learned = False
        # Flag to check if a DMP has been set active
        self.is_dmp_active = False
        # Duration of the planned trajectory (in seconds)
        self.tau = 0.0
        # Dimension of the DMP.
        self.dimensions = None
        # Proportional gains for each of the dimensions of the DMP.
        self.k_gains = None
        # Damping gains for each of the dimensions of the DMP.
        self.d_gains = None
        # Velocity from which to begin the planned trajectory.
        self.starting_velocity = None
        # Threshold in each dimension of the DMP, that the plan must reach
        # before stopping planning.
        self.goal_threshold = None

        # Wait for services.
        learn_dmp_from_demo = rospy.get_param(
            '~learn_dmp_from_demo', 'learn_dmp_from_demo'
        )
        rospy.loginfo("Waiting for '{0}' server".format(learn_dmp_from_demo))
        rospy.wait_for_service(learn_dmp_from_demo)
        self.lfd_client = rospy.ServiceProxy(
            learn_dmp_from_demo, dmp.srv.LearnDMPFromDemo
        )
        rospy.loginfo("Found server '{0}'".format(learn_dmp_from_demo))

        set_active_dmp = rospy.get_param('~set_active_dmp', 'set_active_dmp')
        rospy.loginfo("Waiting for '{0}' server".format(set_active_dmp))
        rospy.wait_for_service(set_active_dmp)
        self.set_active_dmp_client = rospy.ServiceProxy(
            set_active_dmp, dmp.srv.SetActiveDMP
        )
        rospy.loginfo("Found server '{0}'".format(set_active_dmp))

        get_dmp_plan = rospy.get_param('~get_dmp_plan', 'get_dmp_plan')
        rospy.loginfo("Waiting for '{0}' server".format(get_dmp_plan))
        rospy.wait_for_service(get_dmp_plan)
        self.get_dmp_plan_client = rospy.ServiceProxy(get_dmp_plan, dmp.srv.GetDMPPlan)
        rospy.loginfo("Found server '{0}'".format(get_dmp_plan))

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # Time resolution of the demo trajectory (in seconds).
        self.dt_demo = rospy.get_param('~dt_demo', 1.0)

        # Basis functions to use. More complex nonlinear functions require more bases,
        # but too many can cause overfitting. Usually no more than 200 basis functions
        # should be used, or thing start to slow down considerably.
        self.number_of_bases = rospy.get_param('~number_of_bases', 4)

        # Reference frame in which the pose array is specified
        self.reference_frame = rospy.get_param('~reference_frame', None)
        assert self.reference_frame is not None, "A reference frame must be specified."

        # Time from which to begin the planned trajectory (in seconds).
        self.starting_time = rospy.get_param('~starting_time', 0.0)

        # Type of motion to plan for (e.g. grasp from top, place on table, etc.)
        self.primitive_motion = None
        # Trajectory defined by the primitive motion.
        self.demo_trajectory = None

        # Length of the plan segment (in seconds).
        # If set to -1.0 if, it plans until convergence is achieved.
        self.segment_length = rospy.get_param('~segment_length', -1.0)

        # Desired duration of the entire DMP generated movement (scaled to the
        # duration of the demonstrated trajectory,  e.g. a value of 2 will generate
        # a plan twice as long as the demonstrated one).
        # By default is set to be of the same duration as the demonstrated trajectory.
        self.duration = rospy.get_param('~duration', 1.0)

        # Time resolution of the planned trajectory (in seconds).
        self.time_resolution = rospy.get_param('~time_resolution', 1.0)

        # Number of times to numerically integrate when changing acceleration
        # to velocity to position.
        # If time_resolution (dt) is rather large, this should be also large
        # (e.g. 5 for dt = 1).
        self.number_of_integrations = rospy.get_param('~number_of_integrations', 5.0)

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.computed_trajectory = rospy.Publisher(
            "~trajectory", geometry_msgs.msg.PoseArray, queue_size=1
        )

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~start_pose", geometry_msgs.msg.PoseStamped, self.start_pose_cb
        )
        rospy.Subscriber(
            "~goal_pose", geometry_msgs.msg.PoseStamped, self.goal_pose_cb
        )

        # Servers
        self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(
            TaskSpaceParams, self.dynamic_reconfigure_cb
        )

    def dynamic_reconfigure_cb(self, config, level):
        """
        Obtains an update for the dynamic reconfigurable parameters.

        """
        rospy.logdebug(
            "Config set to {0}\n(Bitmask level: {1})".format(
                config.duration, level
            )
        )
        self.duration = config.duration
        self.primitive_motion = config.primitive_motion

        return config

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
            elif state == 'CONFIGURING':
                state = self.configuring_state()
            elif state == 'LEARNING':
                state = self.learning_state()
            elif state == 'SETTING':
                state = self.setting_state()
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
            self.reset_inputs()
            return 'INIT'
        elif self.start_pose and self.goal_pose:
            return 'CONFIGURING'
        else:
            return 'IDLE'

    def configuring_state(self):
        """
        Executes the CONFIGURING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.reset_inputs()
            return 'INIT'
        else:
            if rospy.has_param('~' + self.primitive_motion):
                self.demo_trajectory = rospy.get_param('~' + self.primitive_motion)
                rospy.logdebug(
                    "Demo trajectory set for '{0}'.".format(self.primitive_motion)
                )
            else:
                rospy.logerr(
                    "Demo trajectory for '{0}' is not defined.".format(
                        self.primitive_motion
                    )
                )
                self.reset_inputs()
                return 'INIT'

            # Consistency check (i.e. all points in the trajectory have the same dimension)
            dimensions = [len(ii) for ii in self.demo_trajectory]
            assert numpy.array_equal(
                dimensions, [dimensions[0] for _ in self.demo_trajectory]
            ), "Dimension of points in trajectory must be the same."

            # Dimension of the DMP.
            self.dimensions = dimensions[0]
            # Proportional gains for each of the dimensions of the DMP.
            self.k_gains = rospy.get_param('~k_gains', [100] * self.dimensions)
            # Damping gains for each of the dimensions of the DMP.
            # By default is set to be critically damped.
            self.d_gains = rospy.get_param(
                '~d_gains', map(lambda x: 2.0 * numpy.sqrt(x), self.k_gains)
            )

            # Velocity from which to begin the planned trajectory.
            # By default is set to zero on all dimensions.
            self.starting_velocity = rospy.get_param(
                '~starting_velocity', [0.0] * self.dimensions
            )

            # Threshold in each dimension of the DMP, that the plan must reach
            # before stopping planning.
            self.goal_threshold = rospy.get_param(
                '~goal_threshold', [0.05] * self.dimensions
            )
            return 'LEARNING'

    def learning_state(self):
        """
        Executes the LEARNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.reset_inputs()
            return 'INIT'
        else:
            if self.is_dmp_learned:
                return 'SETTING'

            self.lfd_response = self.learn_dmp_from_demo()
            if self.lfd_response is None:
                rospy.logerr("Could not learn DMP from demonstration.")
                self.event_out.publish('e_failure')
                self.reset_inputs()
                return 'INIT'

            self.is_dmp_learned = True
            rospy.logdebug("DMP from demo has been learned.")
            self.tau = self.duration * self.lfd_response.tau
            return 'SETTING'

    def setting_state(self):
        """
        Executes the CONFIGURING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.reset_inputs()
            return 'INIT'
        else:
            if self.is_dmp_active:
                return 'RUNNING'
            if not self.set_dmp_as_active(self.lfd_response.dmp_list):
                rospy.logerr("Could not set DMP as active.")
                self.event_out.publish('e_failure')
                self.reset_inputs()
                return 'INIT'
            else:
                self.is_dmp_active = True
                return 'RUNNING'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.reset_inputs()
            return 'INIT'
        else:
            trajectory = self.calculate_dmp_plan()
            if trajectory is not None:
                self.computed_trajectory.publish(trajectory)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')
                self.reset_inputs()
                return 'INIT'

        self.start_pose = None
        self.goal_pose = None
        return 'IDLE'

    def reset_inputs(self):
        """
        Resets the component's inputs.

        """
        self.event = None
        self.start_pose = None
        self.goal_pose = None
        self.lfd_response = None
        self.is_dmp_learned = False
        self.is_dmp_active = False

    def calculate_dmp_plan(self):
        """
        Calculates a trajectory based on a DMP, and a start and goal points;
        as a list of poses.

        :return: The DMP plan as a PoseArray message.
        :rtype: geometry_msgs.msg.PoseArray or None

        """
        assert (self.start_pose.header.frame_id == self.goal_pose.header.frame_id),\
            "The frame id of the start and goal pose must be the same.\n'{0}' != '{1}'" \
            "".format(self.start_pose.header.frame_id, self.goal_pose.header.frame_id)
        # get only the joint positions, as a list.
        start_point = pose_stamped_to_list(self.start_pose)
        goal_point = pose_stamped_to_list(self.goal_pose)

        dmp_plan = self.generate_trajectory_plan(start_point, goal_point)
        if dmp_plan is not None:
            rospy.logdebug("DMP plan found.")

            trajectory = dmp_plan_to_pose_array(dmp_plan)
            trajectory.header.stamp = rospy.Time.now()
            trajectory.header.frame_id = self.reference_frame
            return trajectory
        else:
            rospy.logerr("Could not calculate a plan.")
            return None

    def learn_dmp_from_demo(self):
        """
        Learns a DMP for a each DoF, by calling the LearnDMPFromDemo service,
        given the following:
            - demo trajectory,
            - proportional (K) gains for each DoF,
            - derivative (D) gains for each DoF,
            - set of basis functions to approximate the DMP forcing function.

        :return: Response from the LearnDMPFromDemo service. If an exception is
            thrown, None is returned.
        :rtype: dmp.srv.LearnDMPFromDemoResponse or None

        """
        demo_trajectory = dmp.msg.DMPTraj()

        for ii, point in enumerate(self.demo_trajectory):
            dmp_point = dmp.msg.DMPPoint()
            dmp_point.positions = point
            demo_trajectory.points.append(dmp_point)
            demo_trajectory.times.append(self.dt_demo * ii)

        try:
            return self.lfd_client(
                demo_trajectory, self.k_gains, self.d_gains, self.number_of_bases
            )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {0}".format(e))
            return None

    def set_dmp_as_active(self, dmp_list):
        """
        Set a DMP as active for planning by calling the SetActiveDMP service.

        :param dmp_list: DMP for each DoF, intended to be linked together with
            a single phase variable.
        :type dmp_list: dmp.msg.DMPData

        :return: True if no errors are encountered.
        :rtype: bool

        """
        try:
            self.set_active_dmp_client(dmp_list)
            rospy.logdebug("Active DMP has been set.")
            return True
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {0}".format(e))
            return False

    def generate_trajectory_plan(self, start_point, goal_point):
        """
        Generates a planned trajectory from a DMP, by calling the GetDMPPlan service,
        given the following:
            - starting position for each DoF to begin planning from,
            - starting velocity for each DoF to begin planning from,
            - time in seconds at which to begin the planning segment.
            - goal of the plan for each DoF,
            - goal threshold for each DoF,
            - length of the requested plan (in seconds),
            -  time constant to set the length of DMP replay in seconds
               until 95% phase convergence,
            - time resolution at which to plan (in seconds),
            - number of times to loop in numerical integration.

        :param start_point: An n-dimensional point representing the
            start of the trajectory.
        :type start_point: list

        :param goal_point: An n-dimensional point representing the
            goal of the trajectory.
        :type goal_point: list

        :return: Response from the GetDMPPlan service. If an exception is
            thrown, None is returned.
        :rtype: dmp.srv.GetDMPPlanResponse or None

        """
        try:
            return self.get_dmp_plan_client(
                start_point, self.starting_velocity, self.starting_time,
                goal_point, self.goal_threshold, self.segment_length,
                self.tau, self.time_resolution, self.number_of_integrations
            )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {0}".format(e))
            return None


def dmp_plan_to_pose_array(dmp_plan):
    """
    Converts a response from a GetDMPPlan service into a
    PoseArray message.

    :param dmp_plan: The response of a GetDMPPlan service.
    :type dmp_plan: dmp.srv.GetDMPPlanResponse

    :return: The DMP plan as a PoseArray message.
    :rtype: geometry_msgs.msg.PoseArray

    """
    assert (type(dmp_plan) == dmp.srv.GetDMPPlanResponse),\
        "'dmp_plan' must be of type 'GetDMPPlanResponse'."
    trajectory = geometry_msgs.msg.PoseArray()
    poses = [point.positions for point in dmp_plan.plan.points]

    for pose in poses:
        my_pose = geometry_msgs.msg.Pose()
        my_pose.position.x = pose[0]
        my_pose.position.y = pose[1]
        my_pose.position.z = pose[2]
        my_pose.orientation.x = pose[3]
        my_pose.orientation.y = pose[4]
        my_pose.orientation.z = pose[5]
        my_pose.orientation.w = pose[6]
        trajectory.poses.append(my_pose)

    return trajectory


def pose_stamped_to_list(pose):
    """
    Converts a PoseArray message into a seven-dimensional list.

    :param pose: The pose to convert into a list.
    :type pose: geometry_msgs.msg.PoseStamped

    :return: A seven-dimensional list representing the position
        and orientation values of a pose.
    :rtype: list

    """
    assert (type(pose) == geometry_msgs.msg.PoseStamped),\
        "'pose' must be of type 'PoseStamped'."
    return [
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        pose.pose.orientation.x, pose.pose.orientation.y,
        pose.pose.orientation.z, pose.pose.orientation.w
    ]


def main():
    rospy.init_node('dmp_task_space', anonymous=True)
    dmp_task_space = DmpTaskSpace()
    dmp_task_space.start()
