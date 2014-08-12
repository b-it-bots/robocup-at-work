#include <mcr_multi_joint_forward_controller/multi_joint_forward_controller.h>

#include <pluginlib/class_list_macros.h>


namespace position_controllers
{
/**
 * Joint command forward controller for joint positions which accepts
 * brics_actuator joint positions and sends commands to a position interface.
 */
typedef forward_command_controller::MultiJointForwardCommandController<
        brics_actuator::JointPositions,
        hardware_interface::PositionJointInterface>
    MultiJointForwardPositionController;
}

namespace velocity_controllers
{
/**
 * Joint command forward controller for joint velocities which accepts
 * brics_actuator joint velocities and sends commands to a velocity interface.
 */
typedef forward_command_controller::MultiJointForwardCommandController<
        brics_actuator::JointVelocities,
        hardware_interface::VelocityJointInterface>
    MultiJointForwardVelocityController;
}

namespace effort_controllers
{
/**
 * Joint command forward controller for joint efforts which accepts
 * brics_actuator joint torques and sends commands to a effort interface.
 */
typedef forward_command_controller::MultiJointForwardCommandController<
        brics_actuator::JointTorques,
        hardware_interface::EffortJointInterface>
    MultiJointForwardEffortController;
}

PLUGINLIB_EXPORT_CLASS(
        position_controllers::MultiJointForwardPositionController,
        controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(
        velocity_controllers::MultiJointForwardVelocityController,
        controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(
        effort_controllers::MultiJointForwardEffortController,
        controller_interface::ControllerBase)
