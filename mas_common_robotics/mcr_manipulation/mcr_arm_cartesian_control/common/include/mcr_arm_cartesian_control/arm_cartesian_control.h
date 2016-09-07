/*
 * arm_cartesian_control.h
 *
 *  Created on: Nov 29, 2012
 *      Author: matthias
 */

#ifndef ARMCARTESIANCONTROL_H_
#define ARMCARTESIANCONTROL_H_

#include <kdl/kdl.hpp>
#include <kdl/chainiksolver.hpp>

namespace arm_cc
{

class Arm_Cartesian_Control
{
protected:

    KDL::ChainIkSolverVel* ik_solver;

    KDL::Chain* arm_chain;

    std::vector<double> upper_joint_limits;
    std::vector<double> lower_joint_limits;


public:
    Arm_Cartesian_Control(KDL::Chain* arm_chain,
                          KDL::ChainIkSolverVel* ik_solver);

    virtual ~Arm_Cartesian_Control();


    void checkLimits(double dt, KDL::JntArray& joint_positions,
                     KDL::JntArray& jntVel);

    //bool watchdog();

    //void stopMotion();

    void process(double dt, KDL::JntArray& position, KDL::Twist& targetVelocity, KDL::JntArrayVel& out_jnt_velocities);

    void setJointLimits(std::vector<double> lower, std::vector<double> upper);

};

} /* namespace arm_cc */
#endif /* ARMCARTESIANCONTROL_H_ */
