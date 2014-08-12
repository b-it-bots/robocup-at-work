#include <pluginlib/class_list_macros.h>

#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <mcr_trajectory_controller/hardware_interface_adapter.h>

namespace velocity_controllers
{

/**
 * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
 * commands to a \b velocity interface.
 */
typedef joint_trajectory_controller::JointTrajectoryController<
		trajectory_interface::QuinticSplineSegment<double>,
		hardware_interface::VelocityJointInterface> JointTrajectoryController;

}

PLUGINLIB_EXPORT_CLASS(velocity_controllers::JointTrajectoryController, controller_interface::ControllerBase)
