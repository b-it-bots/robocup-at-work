/*
 * ros_arm_cartesian_control.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: matthias
 */

#include "ros_arm_cartesian_control.h"

#include "ros/ros.h"

RosArmCartesianControl::RosArmCartesianControl() {
	tf_listener = 0;
}

RosArmCartesianControl::~RosArmCartesianControl() {

}

void RosArmCartesianControl::jointstateCallback(sensor_msgs::JointStateConstPtr joints) {

	for (unsigned i = 0; i < joints->position.size(); i++) {

		const char* joint_uri = joints->name[i].c_str();

		for (unsigned int j = 0; j < g_arm_chain.getNrOfJoints(); j++) {
			const char* chainjoint =
					g_arm_chain.getSegment(j).getJoint().getName().c_str();

			if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0) {
				g_joint_positions.data[j] = joints->position[i];
				g_joint_positions_initialized[j] = true;
			}
		}
	}
}

void RosArmCartesianControl::ccCallback(geometry_msgs::TwistStampedConstPtr desiredVelocity) {

	for (size_t i = 0; i < g_joint_positions_initialized.size(); i++) {
		if (!g_joint_positions_initialized[i]) {
			std::cout << "joints not initialized" << std::endl;
			return;
		}
	}

	if (!tf_listener) return;

	geometry_msgs::Vector3Stamped linear_in;
	geometry_msgs::Vector3Stamped linear_out;
	linear_in.header = desiredVelocity->header;
	linear_in.vector = desiredVelocity->twist.linear;
	tf_listener->transformVector(root_name, linear_in, linear_out);

	geometry_msgs::Vector3Stamped angular_in;
	geometry_msgs::Vector3Stamped angular_out;
	angular_in.header = desiredVelocity->header;
	angular_in.vector = desiredVelocity->twist.angular;
	tf_listener->transformVector(root_name, angular_in, angular_out);

	targetVelocity.vel.data[0] = linear_out.vector.x;
	targetVelocity.vel.data[1] = linear_out.vector.y;
	targetVelocity.vel.data[2] = linear_out.vector.z;

	targetVelocity.rot.data[0] = angular_out.vector.x;
	targetVelocity.rot.data[1] = angular_out.vector.y;
	targetVelocity.rot.data[2] = angular_out.vector.z;

	t_last_command = ros::Time::now();

	active = true;
}


void RosArmCartesianControl::init_ik_solver() {

	if(ik_solver != 0) {
		return;
	}

	ik_solver = new KDL::ChainIkSolverVel_wdls(g_arm_chain);
	weight_ts.resize(6, 6);
	weight_ts.setIdentity();

	//weight_ts(0, 0) = 1;
	//weight_ts(1, 1) = 1;
	//weight_ts(2, 2) = 10;
	//weight_ts(3, 3) = 0.0001;
	//weight_ts(4, 4) = 0.0001;
	//weight_ts(5, 5) = 0.0001;
	//((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightTS(weight_ts);

	weight_js = (Eigen::MatrixXd::Identity(g_arm_chain.getNrOfJoints(),
			g_arm_chain.getNrOfJoints()));
	//weight_js(0, 0) = 0.5;
	//weight_js(1,1) = 1;
	//weight_js(2,2) = 1;
	//weight_js(3,3) = 1;
	//weight_js(4,4) = 0.1;
	//((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightJS(weight_js);


	((KDL::ChainIkSolverVel_wdls*) ik_solver)->setLambda(10000.0);
}

void RosArmCartesianControl::init_joint_msgs() {
	g_joint_positions_initialized.resize(g_arm_chain.getNrOfJoints(), false);
	jointMsg.velocities.resize(g_arm_chain.getNrOfJoints());
	for (unsigned int i = 0; i < g_arm_chain.getNrOfSegments(); i++) {
		jointMsg.velocities[i].joint_uri =
				g_arm_chain.getSegment(i).getJoint().getName();
		jointMsg.velocities[i].unit = "s^-1 rad";
	}
}

void RosArmCartesianControl::publishJointVelocities(KDL::JntArrayVel& joint_velocities) {

	for (unsigned int i=0; i<joint_velocities.qdot.rows(); i++) {
		jointMsg.velocities[i].value = joint_velocities.qdot(i);
		ROS_DEBUG("%s: %.5f %s", jointMsg.velocities[i].joint_uri.c_str(),
			  jointMsg.velocities[i].value, jointMsg.velocities[i].unit.c_str());
		if (isnan(jointMsg.velocities[i].value)) {
			ROS_ERROR("invalid joint velocity: nan");
			return;
		}
		if (fabs(jointMsg.velocities[i].value) > 1.0) {
			ROS_ERROR("invalid joint velocity: too fast");
			return;
		}
	}
	cmd_vel_publisher.publish(jointMsg);
}


void RosArmCartesianControl::stopMotion() {

	for (unsigned int i = 0; i < jointMsg.velocities.size(); i++) {
		jointMsg.velocities[i].value = 0.0;

	}
	cmd_vel_publisher.publish(jointMsg);
}


bool RosArmCartesianControl::watchdog() {

	double watchdog_time = 0.3;
	if (active==false) {
		return false;
	}

	ros::Time now = ros::Time::now();

	ros::Duration time = (now - t_last_command);

	if ( time > ros::Duration(watchdog_time) ) {
		active = false;
		stopMotion();
		return false;
	}

	return true;
}
