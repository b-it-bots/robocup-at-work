/**
 * @file component_wise_pose_error_monitor_node.h
 * @author Ashok Meenakshi Sundaram (mashoksc@gmail.com)
 * @date June, 2015
 */

#ifndef COMPONENTWISEPOSEERRORMONITORNODE_H_
#define COMPONENTWISEPOSEERRORMONITORNODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <mcr_manipulation_msgs/ComponentWiseCartesianDifference.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <mcr_geometric_relation_monitors/ComponentWisePoseErrorMonitorConfig.h>
#include <math.h>
#include <mcr_monitoring_msgs/ComponentWisePoseErrorMonitorFeedback.h>

/**
 * ROS interface to monitor component wise pose error calculated for two pose stamped messages
 */
class ComponentWisePoseErrorMonitorNode
{

public:
    /**
     * Constructor.
     *
     * @param nh Private node handle.
     */
    ComponentWisePoseErrorMonitorNode(ros::NodeHandle &nh);

    /**
     * Destructor.
     */
    virtual ~ComponentWisePoseErrorMonitorNode();

    /**
     * Call back for dynamic reconfigure topic to decide the current state of the node.
     *
     * @param config The updated configuration.
     */
    void dynamicReconfigCallback(mcr_geometric_relation_monitors::ComponentWisePoseErrorMonitorConfig &config, uint32_t level);

    /**
     * Call back for event_in topic to decide the current state of the node.
     *
     * @param event_command The event string.
     */
    void eventCallback(const std_msgs::String &event_command);

    /**
     * Call back for pose error topic.
     *
     * @param pose_error The pose error calcualed by component wise pose error calculator node.
     */
    void errorCallback(const mcr_manipulation_msgs::ComponentWiseCartesianDifference::ConstPtr &pose_error);

    /**
     * Handles the current state of the node based on the events and data availability.
     */
    void states();

    /**
     * Wait until event e_start is received and then switch to IDLE state.
     */
    void initState();

    /*
     * Wait until data (pose error) is available and then switch to RUNNING state.
     * If event e_stop is sent then switch to INIT state.
     */
    void idleState();

    /*
     * Process the data as per the functionality and publish the corresponding event in the event_out topic.
     * Then switch to IDLE state.
     */
    void runState();

    /*
     * Check if the all the components are error are within the allowed threshold
     */
    bool isComponentWisePoseErrorWithinThreshold();

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
     * Dynamic reconfigure server
     */
    dynamic_reconfigure::Server<mcr_geometric_relation_monitors::ComponentWisePoseErrorMonitorConfig> dynamic_reconfig_server_;

    /**
     * ROS subscriber for the event_in topic
     */
    ros::Subscriber event_sub_;

    /**
     * ROS subscriber for the pose error topic
     */
    ros::Subscriber error_sub_;

    /**
     * ROS publisher to publish the event out message in event_out topic
     */
    ros::Publisher event_pub_;

    /**
     * ROS publisher to publish the feedback message in feedback topic
     */
    ros::Publisher feedback_pub_;

    /**
     * Represents the pose erro data availability in pose error topic
     */
    bool has_pose_error_data_;

    /**
     * Stores the allowed threshold in linear x
     */
    double threshold_linear_x_;

    /**
     * Stores the allowed threshold in linear y
     */
    double threshold_linear_y_;

    /**
     * Stores the allowed threshold in linear z
     */
    double threshold_linear_z_;

    /**
     * Stores the allowed threshold in angular x
     */
    double threshold_angular_x_;

    /**
     * Stores the allowed threshold in angular y
     */
    double threshold_angular_y_;

    /**
     * Stores the allowed threshold in angular z
     */
    double threshold_angular_z_;

    /**
     * Stores the component wise pose error received from the topic
     */
    mcr_manipulation_msgs::ComponentWiseCartesianDifference error_;

    /**
     * Stores the component wise pose error monitor feedback to be published to the topic
     */
    mcr_monitoring_msgs::ComponentWisePoseErrorMonitorFeedback feedback_;

    /**
     * Stores the event out message for the event out topic
     */
    std_msgs::String status_msg_;

    /**
     * Stores the current running state of the node
     */
    States current_state_;

    /**
     * Stores the event in message received from the topic
     */
    std_msgs::String event_in_msg_;

};

#endif /* COMPONENTWISEPOSEERRORMONITORNODE_H_ */
