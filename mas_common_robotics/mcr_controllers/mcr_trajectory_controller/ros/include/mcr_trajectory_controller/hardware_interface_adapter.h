///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef HARDWARE_INTERFACE_ADAPTER_H
#define HARDWARE_INTERFACE_ADAPTER_H

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <joint_trajectory_controller/hardware_interface_adapter.h>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

/**
 * \brief Adapter for a velocity-controlled hardware interface. Maps position
 * errors to velocity commands through a position PID loop.
 *
 * The following is an example configuration of a controller that uses this
 * adapter. Notice the \p gains entry:
 * \code
 * head_controller:
 *   type: "velocity_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint:
 *       trajectory: 0.05
 *       goal: 0.02
 *     head_2_joint:
 *       goal: 0.01
 *
 *   gains:
 *     head_1_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *     head_2_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 * \endcode
 */
template<class State>
class HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, State>
{
	public:
		HardwareInterfaceAdapter() : joint_handles_ptr_(0)
		{
		}

		bool init(std::vector<hardware_interface::JointHandle> &joint_handles,
				ros::NodeHandle &controller_nh)
		{
			// Store pointer to joint handles
			joint_handles_ptr_ = &joint_handles;

			// Initialize PIDs
			pids_.resize(joint_handles.size());
			for (unsigned int i = 0; i < pids_.size(); ++i) {
				// Node handle to PID gains
				ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joint_handles[i].getName());

				// Init PID gains from ROS parameter server
				pids_[i].reset(new control_toolbox::Pid());
				if (!pids_[i]->init(joint_nh)) {
					ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
					return false;
				}
			}

			return true;
		}

		void starting(const ros::Time &time)
		{
			if (!joint_handles_ptr_) {
				return;
			}

			// Reset PIDs, zero effort commands
			for (unsigned int i = 0; i < pids_.size(); ++i) {
				pids_[i]->reset();
				(*joint_handles_ptr_)[i].setCommand(0.0);
			}
		}

		void stopping(const ros::Time &time)
		{
		}

		void updateCommand(const ros::Time & time,
				const ros::Duration &period,
				const State &desired_state,
				const State &state_error)
		{
			const unsigned int n_joints = joint_handles_ptr_->size();

			// Preconditions
			if (!joint_handles_ptr_) {
				return;
			}
			assert(n_joints == state_error.position.size());

			// Update PIDs
			for (unsigned int i = 0; i < n_joints; ++i) {
				const double command = pids_[i]->computeCommand(state_error.position[i], period);
				(*joint_handles_ptr_)[i].setCommand(command);
			}
		}

	private:
		typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
		std::vector<PidPtr> pids_;

		std::vector<hardware_interface::JointHandle> *joint_handles_ptr_;
};

#endif
