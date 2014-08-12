/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Author: Matthias Fueller (matthias.fueller@h-brs.de)
 *
 *  Copyright (c) 2013, Hochschule Bonn-Rhein-Sieg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redstributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Hochschule Bonn-Rhein-Sieg nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * This is the ROS node class for the Cartesian controller.
 * It provides the connection between the ROS environment
 * and the arm_cartesian_control class.
 */

#ifndef ROS_ARM_CARTESIAN_CONTROL_H_
#define ROS_ARM_CARTESIAN_CONTROL_H_

class RosArmCartesianControl {
protected:

	KDL::Chain g_arm_chain;

	std::vector<boost::shared_ptr<urdf::JointLimits> > g_joint_limits;

	KDL::JntArray g_joint_positions;

	std::vector<bool> g_joint_positions_initialized;

	KDL::Twist targetVelocity;

	KDL::ChainIkSolverVel* ik_solver;

	Eigen::MatrixXd weight_ts;

	Eigen::MatrixXd weight_js;

	ros::Publisher cmd_vel_publisher;

	tf::TransformListener *tf_listener;

	bool active = false;

	ros::Time t_last_command;

	brics_actuator::JointVelocities jointMsg;


public:
	RosArmCartesianControl();
	virtual ~RosArmCartesianControl();

	void jointstateCallback(sensor_msgs::JointStateConstPtr joints);

	void ccCallback(geometry_msgs::TwistStampedConstPtr desiredVelocity);

	void init_ik_solver();

	void init_joint_msgs();

	void publishJointVelocities(KDL::JntArrayVel& joint_velocities);

	void stopMotion();
	bool watchdog();
};

#endif /* ROS_ARM_CARTESIAN_CONTROL_H_ */
