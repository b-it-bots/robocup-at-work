#!/usr/bin/env python
"""
Integration test for the 'dynamic_reconfigure_client' node.

"""

import unittest
import rospy
import rostest
import std_msgs.msg
import dynamic_reconfigure.server
import mcr_dynamic_reconfigure_client.cfg.TestNodeConfig as TestNodeConfig

PKG = 'mcr_dynamic_reconfigure_client'


class TestDynamicReconfigureClient(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.wait_for_event = None
        self.event_in_msg = None

        self.int_param = 1
        self.double_param = 1.0

        # publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String)
        self.pub_named_configuration = rospy.Publisher(
            '~configuration_name', std_msgs.msg.String
        )

        self.event_in = rospy.Subscriber(
            '~event_in', std_msgs.msg.String, self.event_in_callback
        )

        self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(TestNodeConfig, self.dynamic_reconfigure_cb)

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.pub_named_configuration.unregister()
        self.event_out.unregister()
        self.event_in.unregister()

    def test_dynamic_reconfigure_client(self):
        """
        Tests whether the dynamic reconifgure client sets parameters of self.dynamic_reconfigure_server
        """

        # wait for node to come up
        rospy.sleep(1.0)

        self.pub_named_configuration.publish("test_node_params_low")
        self.event_out.publish("e_start")

        start_time = rospy.Time.now()

        timeout = rospy.Duration.from_sec(2.0)

        # wait for event_out from node
        while not self.wait_for_event and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)

        self.wait_for_event = None

        self.assertEqual(self.int_param, 0)
        self.assertEqual(self.double_param, 0.0)
        self.assertEqual(self.event_in_msg.data, "e_success")

        self.event_in_msg = None

        self.pub_named_configuration.publish("test_node_params_high")
        self.event_out.publish("e_start")

        start_time = rospy.Time.now()
        while not self.wait_for_event and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)

        self.assertEqual(self.int_param, 10)
        self.assertEqual(self.double_param, 10.0)
        self.assertEqual(self.event_in_msg.data, "e_success")

    def event_in_callback(self, msg):
        self.wait_for_event = True
        self.event_in_msg = msg

    def dynamic_reconfigure_cb(self, config, level):
        self.int_param = config.int_param
        self.double_param = config.double_param
        return config

if __name__ == '__main__':
    rospy.init_node('test_dynamic_reconfigure_client')
    rostest.rosrun(PKG, 'test_dynamic_reconfigure_client', TestDynamicReconfigureClient)
