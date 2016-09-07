/**
 * @file twist_demultiplexer_node.h
 * @author Ashok Meenakshi Sundaram (mashoksc@gmail.com)
 * @date June, 2015
 */

#ifndef TWIST_DEMULTIPLEXER_NODE_H_
#define TWIST_DEMULTIPLEXER_NODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <string.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <mcr_monitoring_msgs/ComponentWisePoseErrorMonitorFeedback.h>

/**
 * ROS interface to demultiplex the twist message to two separate twist messages (for arm and base)
 */
class TwistDemultiplexerNode
{

public:

    /**
     * Constructor.
     *
     * @param nh Private node handle.
     */
    TwistDemultiplexerNode(ros::NodeHandle &nh);

    /**
     * Destructor.
     */
    virtual ~TwistDemultiplexerNode();

    /**
     * Call back for event in message.
     *
     * @param event_command The input event string.
     */
    void eventCallback(const std_msgs::String &event_command);

    /**
     * Call back for input twist message.
     *
     * @param msg The input twist.
     */
    void twistStampedCallback(const geometry_msgs::TwistStamped &msg);

    /**
     * Call back for error monitor feedback.
     *
     * @param error_feedback The error monitor feedback.
     */
    void errorFeedbackCallback(const mcr_monitoring_msgs::ComponentWisePoseErrorMonitorFeedback &error_feedback);

    /**
     * Handles the current state of the node based on the events and data availability.
     */
    void states();

    /**
     * Wait until event e_start is received and then switch to IDLE state.
     */
    void initState();

    /*
     * Wait until data (twist and moitor feedback) is available and then switch to RUNNING state.
     * If event e_stop is sent then switch to INIT state.
     */
    void idleState();

    /*
     * Process the data as per the functionality and publish the corresponding twist in base and arm.
     * Then switch to IDLE state.
     */
    void runState();

    /*
     * Calculate the twist to be demultiplexed for arm and base as given in the launch configuration
     * If error monitor feedback is within tolerance then make it to zero.
     * Then switch to IDLE state.
     *
     * @param twist_as_array Stores the twist array to be updated
     * @param is_twist_part_enabled Stores the boolean to say which parts of twist are enabled
     */
    void demultiplexTwist(double *twist_as_array, bool *is_twist_part_enabled);

private:

    /**
     * An enum to handle the current state of the node
     */
    enum States
    {
        INIT, /**< INIT state while waiting for start event */
        IDLE, /**< IDLE state while waiting for the data. */
        RUNNING /**< RUNNING state while processing the data. */
    };

private:

    /**
     * ROS Node handler for the node.
     */
    ros::NodeHandle node_handler_;

    /**
     * ROS subscriber for the event_in topic.
     */
    ros::Subscriber event_sub_;

    /**
     * ROS subscriber for the twist input topic.
     */
    ros::Subscriber twist_sub_;

    /**
     * ROS subscriber for the error monitor feedback topic.
     */
    ros::Subscriber error_feedback_sub_;

    /**
     * ROS publisher for the arm twist.
     */
    ros::Publisher arm_twist_stamped_pub_;

    /**
     * ROS publisher for the base twist.
     */
    ros::Publisher base_twist_pub_;

    /**
     * Stores the input twist message
     */
    geometry_msgs::TwistStamped input_twist_;

    /**
     * Stores the error monitor feedback message
     */
    mcr_monitoring_msgs::ComponentWisePoseErrorMonitorFeedback error_feedback_msg_;

    /**
     * Represents if twist data is available
     */
    bool has_twist_data_;

    /**
     * Represents if error monitor feedback data is available
     */
    bool has_error_feedback_data_;

    /**
     * Stores the arm tf for the arm twist
     */
    std::string arm_tf_;

    /**
     * Stores the event input message
     */
    std_msgs::String event_in_msg_;

    /**
     * Stores the current state of the node
     */
    States current_state_;

    /**
     * Represents if error monitor node is enabled for processing
     */
    bool is_error_monitor_enabled_;

    /**
     * Stores the arm twist to be published
     */
    geometry_msgs::TwistStamped arm_twist_;

    /**
     * Stores the base to be published
     */
    geometry_msgs::Twist base_twist_;

    /**
     * Stores the number of parts avaialble in a twist message (linear - 3 and angular - 3)
     */
    static const int NO_OF_PARTS_IN_TWIST = 6;

    /**
     * Represents the parts of twist message that are enabled for base
     */
    bool is_twist_part_enabled_in_base_[NO_OF_PARTS_IN_TWIST];

    /**
     * Represents the parts of twist message that are enabled for arm
     */
    bool is_twist_part_enabled_in_arm_[NO_OF_PARTS_IN_TWIST];

    /**
     * Represents the parts of twist message that are within error tolerance
     */
    bool is_error_part_within_tolerance_[NO_OF_PARTS_IN_TWIST];

    /**
     * Represents the input twist in an array
     */
    double input_twist_array_[NO_OF_PARTS_IN_TWIST];

    /**
     * Represents the base twist in an array
     */
    double base_twist_array_[NO_OF_PARTS_IN_TWIST];

    /**
     * Represents the arm twist in an array
     */
    double arm_twist_array_[NO_OF_PARTS_IN_TWIST];

};

#endif /* TWIST_DEMULTIPLEXER_NODE_H_ */