#!/usr/bin/python

ID = '/rosnode'

import rospy

import os
import sys
import socket
import time
import xmlrpclib
import rospy

from diagnostic_msgs.msg import *
from optparse import OptionParser

import roslib.scriptutil as scriptutil 

class ROSNodeException(Exception):
    """
    rosnode base exception type
    """
    pass
class ROSNodeIOException(ROSNodeException):
    """
    Exceptions for communication-related (i/o) errors, generally due to Master or Node network communication issues.
    """
    pass
    
def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSNodeException("remote call failed: %s"%msg)
    return val


def get_api_uri(master, caller_id):
    try:
        code, msg, caller_api = master.lookupNode(ID, caller_id)
        if code != 1:
            return None
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
    return caller_api

def rosnode_ping(node_name):
    max_count=1
  
    master = scriptutil.get_master()
    
    node_api = get_api_uri(master,node_name)
    if not node_api:
        #print >> sys.stderr, "cannot ping [%s]: unknown node"%node_name
        #print 'NODE IS DOWN'
        return False

    timeout = 3.

    
    #print "pinging %s with a timeout of %ss"%(node_name, timeout)

    socket.setdefaulttimeout(timeout)
    node = xmlrpclib.ServerProxy(node_api)
    lastcall = 0.
    acc = 0.
    count = 0
    try:
        while True:
            try:
                start = time.time()
                pid = _succeed(node.getPid(ID))
                end = time.time()
                
                dur = (end-start)*1000.
                acc += dur
                count += 1
                
                #print 'NODE IS UP'
                #print "xmlrpc reply from %s\ttime=%fms"%(node_api, dur)
                # 1s between pings
            except socket.error:
                #print 'NODE IS DOWN'
                #print >> sys.stderr, "connection to [%s] timed out"%node_name
                return False
            if max_count and count >= max_count:
                break
            #time.sleep(1.0)
    except KeyboardInterrupt:
        pass
            
    return True


def main():
    rospy.init_node('mcr_node_status')
    
    filename = rospy.get_param('~NodeListFilename')
       
    file = open(filename)

    node_list_to_check = []
    while 1:
      line = file.readline()
      if not line:
        break
      line = line[:-1]
      node_list_to_check.append(line)
      
    pub_diag = rospy.Publisher('/diagnostics', DiagnosticArray)
    print "running..."
    
    while not rospy.is_shutdown():
        diag_list = DiagnosticArray()
        diag_list.header.stamp = rospy.Time.now()
        
        for item in node_list_to_check:
            diag_msg = DiagnosticStatus()

            if rosnode_ping(item) == True:
              diag_msg.level = DiagnosticStatus.OK
              diag_msg.name = 'mcr_node_status_' + item
              diag_msg.message = "node is UP"
            else:
              diag_msg.level = DiagnosticStatus.ERROR
              diag_msg.name = 'mcr_node_status_' + item
              diag_msg.message = "node is DOWN"
            
            diag_list.status.append(diag_msg)
            
        pub_diag.publish(diag_list)
        rospy.sleep(1.0)

                        
            
            
            
        
    
    
    
    

