#ifndef MULTI_JOINT_FORWARD_COMMAND_CONTROLLER_H
#define MULTI_JOINT_FORWARD_COMMAND_CONTROLLER_H

#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>


namespace forward_command_controller {

/**
 * The MultiJointForwardCommandController allows to forward a set of joint
 * commands for several joints directly to the hardware interface of a robot.
 *
 * The template parameter RosInterfaceT specifies the type of input message that
 * the controller receives. For each new message type the according callback
 * must be specialized. Currently, the brics_actuator messages are supported.
 * The template parameter HardwareInterfaceT is a subclass of
 * hardware_interface::HardwareInterface.
 */
template <class RosInterfaceT, class HardwareInterfaceT>
class MultiJointForwardCommandController
        : public controller_interface::Controller<HardwareInterfaceT> {
    public:
        /**
         * Ctor.
         */
        MultiJointForwardCommandController();

        /**
         * Dtor.
         */
        virtual ~MultiJointForwardCommandController();

        /**
         * Initialize the controller. This function is not real-time safe.
         *
         * @param hw A pointer to the HardwareInterface.
         */
        bool init(HardwareInterfaceT *hw, ros::NodeHandle &root_nh,
                ros::NodeHandle &controller_nh);

        /**
         * Start the controller. This function is real-time safe.
         *
         * @param time The reference time when the controller starts running.
         */
        void starting(const ros::Time &time);

        /**
         * Update the controller. This function is real-time safe.
         *
         * @param time The current time when the update of the controller is
         * executed.
         *
         * @param period The duration from the last update to the current
         * update.
         */
        void update(const ros::Time &time, const ros::Duration &period);


    private:
        /**
         * Copy ctor.
         */
        MultiJointForwardCommandController(
                const MultiJointForwardCommandController &other);

        /**
         * Assignment operator.
         */
        MultiJointForwardCommandController &operator=(
                const MultiJointForwardCommandController &other);


    private:
        /**
         * The ROS subscriber for the input topic.
         */
        ros::Subscriber sub_command_;

        /**
         * The commanded joint values which have been received last.
         */
        std::vector<double> command_;

        /**
         * The joint handles from the hardware interface to which the commands
         * are sent.
         */
        std::vector<hardware_interface::JointHandle> joints_;
};

template <class RosInterfaceT>
void commandCallback(const boost::shared_ptr<RosInterfaceT const> &msg,
        std::vector<double> *command);

}

#include "multi_joint_forward_controller.impl"

#endif
