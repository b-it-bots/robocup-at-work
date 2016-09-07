#include <mcr_multi_joint_forward_controller/multi_joint_forward_controller.h>

#include <pluginlib/class_list_macros.h>


namespace position_controllers
{
/**
 * Joint command forward controller for joint positions which accepts
 * brics_actuator joint positions and sends commands to a position interface.
 */
typedef forward_command_controller::MultiJointForwardCommandController <
        brics_actuator::JointPositions,
        hardware_interface::PositionJointInterface >
    MultiJointForwardBricsPositionController;

/**
 * Joint command forward controller for joint positions which accepts
 * Float64MultiArray joint positions and sends commands to a position interface.
 */
typedef forward_command_controller::MultiJointForwardCommandController <
        std_msgs::Float64MultiArray,
        hardware_interface::PositionJointInterface >
    MultiJointForwardPositionController;
}

namespace velocity_controllers
{
/**
 * Joint command forward controller for joint velocities which accepts
 * brics_actuator joint velocities and sends commands to a velocity interface.
 */
typedef forward_command_controller::MultiJointForwardCommandController <
        brics_actuator::JointVelocities,
        hardware_interface::VelocityJointInterface >
    MultiJointForwardBricsVelocityController;

/**
 * Joint command forward controller for joint velocities which accepts
 * Float64MultiArray joint velocities and sends commands to a velocity interface.
 */
typedef forward_command_controller::MultiJointForwardCommandController <
        std_msgs::Float64MultiArray,
        hardware_interface::VelocityJointInterface >
    MultiJointForwardVelocityController;
}

namespace effort_controllers
{
/**
 * Joint command forward controller for joint efforts which accepts
 * brics_actuator joint torques and sends commands to a effort interface.
 */
typedef forward_command_controller::MultiJointForwardCommandController <
        brics_actuator::JointTorques,
        hardware_interface::EffortJointInterface >
    MultiJointForwardBricsEffortController;

/**
 * Joint command forward controller for joint efforts which accepts
 * Float64MultiArray joint torques and sends commands to a effort interface.
 */
typedef forward_command_controller::MultiJointForwardCommandController <
        std_msgs::Float64MultiArray,
        hardware_interface::EffortJointInterface >
    MultiJointForwardEffortController;
}


PLUGINLIB_EXPORT_CLASS(
    position_controllers::MultiJointForwardBricsPositionController,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(
    velocity_controllers::MultiJointForwardBricsVelocityController,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(
    effort_controllers::MultiJointForwardBricsEffortController,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(
    position_controllers::MultiJointForwardPositionController,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(
    velocity_controllers::MultiJointForwardVelocityController,
    controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(
    effort_controllers::MultiJointForwardEffortController,
    controller_interface::ControllerBase)
