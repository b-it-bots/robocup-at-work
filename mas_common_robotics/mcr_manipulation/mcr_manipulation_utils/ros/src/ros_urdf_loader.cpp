/*
 * ros_urdf_loader.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: matthias
 */

#include "mcr_manipulation_utils/ros_urdf_loader.h"

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

ROS_URDF_Loader::ROS_URDF_Loader() {

}

ROS_URDF_Loader::~ROS_URDF_Loader() {

}

bool ROS_URDF_Loader::loadModel(ros::NodeHandle& node_handle,
		const std::string &root_name,
		const std::string &tip_name,
		KDL::Chain& out_arm,
		std::vector<boost::shared_ptr<urdf::JointLimits> >& out_joint_limits) {
	urdf::Model robot_model;

	std::string xml_string;
	std::string urdf_xml, full_urdf_xml;
	node_handle.param("urdf_xml", urdf_xml, std::string("robot_description"));
	node_handle.searchParam(urdf_xml, full_urdf_xml);
	TiXmlDocument xml;
	ROS_DEBUG("Reading xml file from parameter server\n");
	std::string result;
	if (node_handle.getParam(full_urdf_xml, result))
		xml.Parse(result.c_str());
	else {
		ROS_FATAL(
				"Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
		return false;
	}
	xml_string = result;
	TiXmlElement *root_element = xml.RootElement();
	TiXmlElement *root = xml.FirstChildElement("robot");
	if (!root || !root_element) {
		ROS_FATAL("Could not parse the xml from %s\n", urdf_xml.c_str());
		return false;
	}
	robot_model.initXml(root);

	KDL::Tree tree;
	kdl_parser::treeFromUrdfModel(robot_model, tree);

	if (!tree.getChain(root_name, tip_name, out_arm)) {
		ROS_FATAL("Could not extract chain");
		return false;
	}

	out_joint_limits.clear();
	for (unsigned int i = 0; i < out_arm.getNrOfJoints(); i++) {

		std::string name = out_arm.getSegment(i).getJoint().getName();

		boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(name);

		out_joint_limits.push_back(joint->limits);

	}

	return true;
}
