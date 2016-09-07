#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import std_msgs.msg
import yaml
import dynamic_reconfigure.client


class DynamicReconfigureClient(object):
    """
    Sets dynamic reconifgure parameters for sets of nodes
    Loads named configurations from a yaml file
    Sets parameters for each node in the named configuration published
    on ~configuration_name
    Configuration file can be loaded during runtime if empty message is publish to
    ~reload_config
    """
    def __init__(self):
        self.configuration_name = None
        self.event = None

        self.config_file_name = rospy.get_param('~config_file')

        self.named_configurations = yaml.load(open(self.config_file_name, 'r'))

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)

        # Subscribers
        rospy.Subscriber(
            '~event_in', std_msgs.msg.String,
            self.event_in_cb
        )
        rospy.Subscriber(
            '~configuration_name', std_msgs.msg.String,
            self.configuration_name_cb
        )
        rospy.Subscriber(
            '~reload_config', std_msgs.msg.Empty,
            self.reload_config_cb
        )

    def start(self):
        """
        Starts the state machine
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

    def configuration_name_cb(self, msg):
        """
        Obtains configuration name to be set
        """
        self.configuration_name = msg.data

    def event_in_cb(self, msg):
        """
        Obtains an event for dynamic reconfiguration

        """
        self.event = msg.data

    def reload_config_cb(self, msg):
        """
        Reloads named configurations from file
        """
        self.named_configurations = yaml.load(open(self.config_file_name, 'r'))

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
            self.event_out.publish("e_stopped")
            self.event = None
            self.configuration_name = None
            return 'INIT'
        elif self.configuration_name:
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
            self.configuration_name = None
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            if self.configuration_name in self.named_configurations.keys():
                if self.set_all_parameters(self.named_configurations[self.configuration_name]):
                    self.event_out.publish("e_success")
                else:
                    self.event_out.publish("e_failure")
                    self.event = None
                    self.configuration_name = None
                    return 'INIT'
            else:
                self.event_out.publish("e_failure")
                self.event = None
                self.configuration_name = None
                return 'INIT'
            self.event = None
            self.configuration_name = None
            return 'IDLE'

    def set_all_parameters(self, params):
        """
        set dynamic reconfigure parameters for all nodes listed under the named configuration
        """
        for key, value in params.items():
            if not self.set_node_parameters(key, value):
                rospy.logerr("Could not set parameters for {0}".format(key))
                return False
        return True

    def set_node_parameters(self, node_name, params):
        """
        create dynamic reconfigure client and set params for a single node
        """

        try:
            client = dynamic_reconfigure.client.Client(node_name, timeout=1.5)
        except Exception, e:
            rospy.logerr("Service {0} does not exist".format(node_name + '/set_parameters'))
            return False
        try:
            config = client.update_configuration(params)
        except Exception as e:
            rospy.logerr("Error: %s", str(e))
            return False
        return True


def main():
    rospy.init_node('dynamic_reconfigure_client', anonymous=True)
    dynamic_reconfigure_client = DynamicReconfigureClient()
    dynamic_reconfigure_client.start()
