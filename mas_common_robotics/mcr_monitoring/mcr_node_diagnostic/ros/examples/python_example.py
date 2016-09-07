#!/usr/bin/env python

import roslib; roslib.load_manifest('mcr_node_diagnostic')

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import mcr.diag

if __name__ == '__main__':
    rospy.init_node('mcr_diag_testnode')

    rospy.sleep(1)


    print("Node observes that no IK service is available")
    mcr.diag.error("InverseKinematics", mcr.diag.COMMUNICATION, "IK solver is missing", {'key_value_pairs':'for more information'})
 
    rospy.sleep(10)

    print("Node observes that IK service is back")
    mcr.diag.reset("InverseKinematics")
