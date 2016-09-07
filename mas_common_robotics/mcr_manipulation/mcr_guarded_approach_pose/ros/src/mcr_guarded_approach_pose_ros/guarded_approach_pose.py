#!/usr/bin/env python
"""
This component moves a manipulator in Cartesian space until a contact is detected.
It uses two nodes:
  * (mcr_arm_motions) cartesian_motion_node and
  * (mcr_twitch_detection) twitch_detector.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for both components and starts/stops them accordingly.

"""
#-*- encoding: utf-8 -*-

import rospy
import std_msgs.msg
import geometry_msgs.msg


class GuardedApproachPose(object):
    """
    Components that move the arm in a planned motion.

    """
    def __init__(self):
        # params
        self.started_components = False
        self.desired_twist = None
        self.event = None

        # the frame w.r.t. the desired twist is specified
        self.twist_frame = rospy.get_param('~twist_frame', None)
        assert self.twist_frame is not None, "A frame must be specified for the desired twist."

        # linear components of the desired twist
        self.twist_linear_x = rospy.get_param('~twist_linear_x', 0.0)
        self.twist_linear_y = rospy.get_param('~twist_linear_y', 0.0)
        self.twist_linear_z = rospy.get_param('~twist_linear_z', 0.0)

        # angular components of the desired twist
        self.twist_angular_x = rospy.get_param('~twist_angular_x', 0.0)
        self.twist_angular_y = rospy.get_param('~twist_angular_y', 0.0)
        self.twist_angular_z = rospy.get_param('~twist_angular_z', 0.0)

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.start_twitch_detector = rospy.Publisher(
            '~start_twitch_detector', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.start_cartesian_motion = rospy.Publisher(
            '~start_cartesian_motion', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.pub_desired_twist = rospy.Publisher(
            '~desired_twist', geometry_msgs.msg.TwistStamped, queue_size=1
        )

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)

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
            if state == 'CONFIGURING':
                state = self.configuring_state()
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
            return 'CONFIGURING'
        else:
            return 'INIT'

    def configuring_state(self):
        """
        Executes the CONFIGURING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        self.desired_twist = geometry_msgs.msg.TwistStamped()
        self.desired_twist.header.frame_id = self.twist_frame
        self.desired_twist.twist.linear.x = self.twist_linear_x
        self.desired_twist.twist.linear.y = self.twist_linear_y
        self.desired_twist.twist.linear.z = self.twist_linear_z
        self.desired_twist.twist.angular.x = self.twist_angular_x
        self.desired_twist.twist.angular.y = self.twist_angular_y
        self.desired_twist.twist.angular.z = self.twist_angular_z

        return 'RUNNING'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.toggle_components()
            self.event = None
            return 'INIT'
        else:
            self.toggle_components()
            return 'RUNNING'

    def toggle_components(self):
        """
        Starts or stops (depending on the events) the necessary components.

        """
        if self.event == 'e_stop':
            self.start_twitch_detector.publish('e_stop')
            self.start_cartesian_motion.publish('e_stop')
            self.event_out.publish('e_failure')
            self.started_components = False

        if self.event == 'e_collision':
            self.start_twitch_detector.publish('e_stop')
            self.start_cartesian_motion.publish('e_stop')
            self.event_out.publish('e_success')
            self.started_components = False

        if self.event == 'e_start' and not self.started_components:
            self.desired_twist.header.stamp = rospy.Time.now()
            self.pub_desired_twist.publish(self.desired_twist)
            self.start_twitch_detector.publish('e_start')
            self.start_cartesian_motion.publish('e_start')
            self.started_components = True


def main():
    rospy.init_node("guarded_approach_pose", anonymous=True)
    guarded_approach_pose = GuardedApproachPose()
    guarded_approach_pose.start()
