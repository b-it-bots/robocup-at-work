/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * twist_to_motion_direction_conversion_node.h
 *
 *  Created on: Mar 28, 2015
 *      Author: fred
 */

#ifndef MCR_COMMON_CONVERTERS_TWIST_TO_MOTION_DIRECTION_CONVERSION_NODE_H_
#define MCR_COMMON_CONVERTERS_TWIST_TO_MOTION_DIRECTION_CONVERSION_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <string>

#include <mcr_common_converters/motion_direction_calculation.h>


class TwistToMotionDirectionConversionNode
{
public:
    TwistToMotionDirectionConversionNode();
    ~TwistToMotionDirectionConversionNode();

    void update();

    enum State {INIT, RUN};

private:
    void twistCallback(const geometry_msgs::TwistPtr &msg);
    void eventCallback(const std_msgs::StringPtr &msg);

    void computeMotionDirectionAndPublish();


    ros::Subscriber sub_twist_;
    ros::Subscriber sub_event_;

    ros::Publisher pub_pose_;
    ros::Publisher pub_point_;

    std::string frame_id_;
    double distance_to_frame_;

    geometry_msgs::Twist twist_msg_;
    bool twist_msg_received_;

    std_msgs::String event_msg_;
    bool event_msg_received_;

    State current_state_;
};

#endif /* MCR_COMMON_CONVERTERS_TWIST_TO_MOTION_DIRECTION_CONVERSION_NODE_H_ */
