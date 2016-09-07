'''
Created on May 23, 2013

@author: matthias fueller (matthias.fueller@h-brs.de
'''

import roslib; roslib.load_manifest('mcr_node_diagnostic')

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


'''
static variables
'''

_pub = rospy.Publisher('/diagnostics', DiagnosticArray)

_STATUS_OK = 0
_STATUS_WARN = 1
_STATUS_ERROR = 2


SOFTWARE = "software"
HARDWARE = "hardware"
COMMUNICATION = "communication"
INTERACTION = "interaction"
ALGORITHMIC = "algorithmic"

diag_msgs = []

def _diag(ID, category, state, msg='', key_value={}):

    array = DiagnosticArray()
    full_name = "mcr_diag" + rospy.get_name() + "/" + ID + "/"
    full_name = full_name.replace("//","/")
    s1 = DiagnosticStatus(name = full_name, level = state, message = msg)
    values = []
    for k, v in key_value.items():
        values.append(KeyValue(key=k, value=v))
    values.append(KeyValue(key='failure_category', value=category))
    s1.values = values
    array.status = [ s1 ]
    _pub.publish(array)
    if (s1.level != _STATUS_OK):
        diag_msgs.append(s1)

def reset(ID, category='', msg='', key_value={}):
    _diag(ID, category, _STATUS_OK, msg, key_value)

def warn(ID, category='', msg='', key_value={}):
    _diag(ID, category, _STATUS_WARN, msg, key_value)

def error(ID, category='', msg='', key_value={}):
    _diag(ID, category, _STATUS_ERROR, msg, key_value)

def resetall():
    for item in diag.diag_msgs:
        _diag.reset(item.name) 
    
        