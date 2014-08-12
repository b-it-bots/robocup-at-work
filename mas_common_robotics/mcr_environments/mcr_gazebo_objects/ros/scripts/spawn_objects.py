#!/usr/bin/python
import sys

import rospy
import os

from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import tf.transformations as tft

if __name__ == "__main__":
    ### wait for gazebo world beeing loaded
    rospy.loginfo("Wait for service <</gazebo/get_world_properties>>")
    rospy.wait_for_service('/gazebo/get_world_properties', 300)
    
    world_loaded = False
    while not world_loaded:    
        srv_world_infos = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    
        try:
            req = GetWorldPropertiesRequest()
            res = srv_world_infos(req)
    
            for item in res.model_names:
                if item == 'world':
                    world_loaded = True
                    break
        except rospy.ServiceException, e:
            print "Service call <</gazebo/get_world_properties>> failed: %s"%e
        
        rospy.sleep(1)
        
    rospy.loginfo("Arena loaded successfully. Start loading objects ...")

    
    # get object information
    param_obj_preffix = "/simulation/objects"
    object_names = rospy.get_param(param_obj_preffix)
    
    for obj_name in object_names:

        model_type = rospy.get_param(param_obj_preffix + "/" + obj_name + "/model_type")
        object_type = rospy.get_param(param_obj_preffix + "/" + obj_name + "/object_type")
        orientation = rospy.get_param(param_obj_preffix + "/" + obj_name + "/orientation")
        position = rospy.get_param(param_obj_preffix + "/" + obj_name + "/position")
        
        # convert rpy to quaternion for Pose message
        quaternion = tft.quaternion_from_euler(orientation[0], orientation[1], orientation[2])

        object_pose = Pose()
        object_pose.position.x = float(position[0])
        object_pose.position.y = float(position[1])
        object_pose.position.z = float(position[2])
        object_pose.orientation.x = quaternion[0]
        object_pose.orientation.y = quaternion[1]
        object_pose.orientation.z = quaternion[2]
        object_pose.orientation.w = quaternion[3]

        file_localition = roslib.packages.get_pkg_dir('mcr_gazebo_objects') + '/ros/urdf/' + object_type + '.' + model_type

        # call gazebo service to spawn model (see http://ros.org/wiki/gazebo)
        if model_type == "urdf":
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            file_xml = open(file_localition)
            xml_string=file_xml.read()

        elif model_type == "urdf.xacro":
            p = os.popen("rosrun xacro xacro.py " + file_localition)
            xml_string = p.read()   
            p.close()
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

        elif model_type == "model":
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_gazebo_model', SpawnModel)
            file_xml = open(file_localition)
            xml_string=file_xml.read()
        else:
            print 'Error: Model type not know. model_type = ' + model_type
            sys.exit()


        req = SpawnModelRequest()
        req.model_name = obj_name # model name from command line input
        req.model_xml = xml_string
        req.initial_pose = object_pose

        res = srv_spawn_model(req)
    
        # evaluate response
        if res.success == True:
            print " %s model spawned succesfully. status message = "% object + res.status_message 
        else:
            print "Error: model %s not spawn. error message = "% object + res.status_message

