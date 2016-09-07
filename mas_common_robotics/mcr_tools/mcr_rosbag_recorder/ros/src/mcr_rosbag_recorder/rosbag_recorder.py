#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import std_msgs.msg
import os
import os.path
import signal
import subprocess
import datetime
import yaml


class RosbagRecorder(object):
    """
    starts and stops rosbag record based on event_in
    """
    def __init__(self):
        self.event = None
        topics_file = rospy.get_param('~topics')
        self.file_path = rospy.get_param('~file_path')
        self.file_prefix = rospy.get_param('~file_prefix')

        # get list of topics from yaml file
        stream = open(topics_file, 'r')
        self.topics = yaml.load(stream)
        self.rosbag_process = None

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)

        # Subscribers
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
            return 'RUNNING'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.stop_recording()
            self.event_out.publish('e_stopped')
            self.event = None
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        self.start_recording()
        self.event_out.publish("e_started")
        self.event = None

        return 'IDLE'

    def start_recording(self):
        today = datetime.datetime.today()
        subfilename = '{:04d}'.format(today.year) + '_' + '{:02d}'.format(today.month) + \
                      '_' + '{:02d}'.format(today.day) + '_' + '{:02d}'.format(today.hour) + \
                      '-' + '{:02d}'.format(today.minute)
        filename = self.file_prefix + '_' + subfilename + '.bag'
        if not os.path.isdir(os.path.expanduser(self.file_path)):
            os.mkdir(os.path.expanduser(self.file_path))
        fullpath = os.path.expanduser(os.path.join(self.file_path, filename))
        rosbag_run_string = 'rosbag record ' + self.topics + ' -O ' + fullpath
        args = rosbag_run_string.split(' ')
        self.rosbag_process = subprocess.Popen(args)

    def stop_recording(self):
        if self.rosbag_process:
            self.terminate_process_and_children(self.rosbag_process)
            self.rosbag_process = None
            rospy.loginfo("Killed rosbag process")

    def terminate_process_and_children(self, p):
        """
        Code by Sergey Alexandrov: http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        """
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
                os.kill(int(pid_str), signal.SIGINT)
        p.terminate()


def main():
    rospy.init_node('rosbag_recorder', anonymous=True)
    rosbag_recorder = RosbagRecorder()
    rosbag_recorder.start()
