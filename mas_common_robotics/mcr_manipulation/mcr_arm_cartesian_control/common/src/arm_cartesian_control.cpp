/*
 * arm_cartesian_control.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: matthias
 */

#include "arm_cartesian_control.h"

#include <iostream>

namespace arm_cc {

Arm_Cartesian_Control::Arm_Cartesian_Control(KDL::Chain* arm_chain,
		KDL::ChainIkSolverVel* ik_solver) {
	this->arm_chain = arm_chain;
	this->ik_solver = ik_solver;
}

Arm_Cartesian_Control::~Arm_Cartesian_Control() {

}

void Arm_Cartesian_Control::checkLimits(
		double dt, KDL::JntArray& joint_positions,
		KDL::JntArray& jntVel) {

	if (upper_joint_limits.size() < arm_chain->getNrOfJoints()
			|| lower_joint_limits.size() < arm_chain->getNrOfJoints()) {
	        std::cout << "No Joint limits defined";
		return;
	}

	double limit_range = 0.1;

	for (int i=0; i<jntVel.data.rows(); i++) {
		double pos = joint_positions.data(i);
		double vel = jntVel.data(i);

		double fpos = pos + vel * dt * 2;

		double upper_limit = this->upper_joint_limits[i];
		double lower_limit = this->lower_joint_limits[i];

		double upper_clearance = upper_limit - fpos;
		double lower_clearance = lower_limit - fpos;

		double limit_vel = vel;


		if (fabs(upper_clearance) < limit_range*2  && vel > 0) {
			limit_vel *= upper_clearance;
		}
		if (fabs(lower_clearance) < limit_range*2  && vel < 0) {
			limit_vel *= lower_clearance;
		}

		if (fpos > upper_limit - limit_range && vel > 0) {
			limit_vel = 0.0;
		        std::cout << "Upper limit reached for joint " << i << std::endl;
 		} else if (fpos < lower_limit + limit_range && vel < 0) {
 			limit_vel = 0.0;
			std::cout << "Lower limit reached for joint " << i << std::endl;
		}

		jntVel.data(i) = limit_vel;

	}
}


void Arm_Cartesian_Control::process(
		double dt,
		KDL::JntArray& joint_positions,
		KDL::Twist& targetVelocity, KDL::JntArrayVel& out_jnt_velocities) {

	out_jnt_velocities.q.data = joint_positions.data;

	double max_lin_frame_velocitiy = 0.1;  // m/s
	double max_joint_vel = 0.25; // radian/s
	double eps_velocity = 0.0001;

	// calc Jacobian
	//KDL::Jacobian jacobian(arm.getNrOfJoints());
	//jnt2jac->JntToJac(joint_positions, jacobian);

	// calc absolute relative velocity; used for joint velocitiy reduction
	double linear_frame_vel = targetVelocity.vel.Norm();
	double angular_frame_vel = targetVelocity.rot.Norm();

	if (linear_frame_vel < eps_velocity && angular_frame_vel < eps_velocity) {
		out_jnt_velocities.qdot.data.setZero();
		return;
	}


	if (linear_frame_vel > max_lin_frame_velocitiy) {
		//reduce target velocity to maximum limit
		targetVelocity.vel.data[0] *= (max_lin_frame_velocitiy / linear_frame_vel) ;
		targetVelocity.vel.data[1] *= (max_lin_frame_velocitiy / linear_frame_vel) ;
		targetVelocity.vel.data[2] *= (max_lin_frame_velocitiy / linear_frame_vel) ;
	}


	// calc joint velocitites
	KDL::JntArray jntVel(arm_chain->getNrOfJoints());

        ik_solver->CartToJnt(joint_positions, targetVelocity, jntVel);


	// limit joint velocitied
	if (jntVel.data.norm() > 0.01) {
		double max_velocity = jntVel.data.cwiseAbs().maxCoeff();
		if (max_velocity > max_joint_vel) {
			jntVel.data /= max_velocity;
			jntVel.data *= max_joint_vel;
		}
	} else {
		jntVel.data.Zero(arm_chain->getNrOfJoints());
	}


	//check for joint limits
	checkLimits(dt, joint_positions, jntVel);

	out_jnt_velocities.qdot.data = jntVel.data;

}

void Arm_Cartesian_Control::setJointLimits(std::vector<double> lower, std::vector<double> upper) {

	assert(lower.size() == arm_chain->getNrOfJoints());
	assert(upper.size() == arm_chain->getNrOfJoints());

	this->lower_joint_limits = lower;
	this->upper_joint_limits = upper;
}







} /* namespace arm_cc */
