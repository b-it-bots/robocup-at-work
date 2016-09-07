/*********************************************************************
 * Software License Agreement (GPLv3 License)
 *
 *  Copyright (c) 2015, Hochschule Bonn-Rhein-Sieg.
 *  All rights reserved.
 *
 *********************************************************************/
/**
 * Author: Shehzad Ahmed
 */
#ifndef MCR_INTERPOLATION_PLANNER_MOVEIT_INTERPOLATION_PLANNER_INTERFACE_H
#define MCR_INTERPOLATION_PLANNER_MOVEIT_INTERPOLATION_PLANNER_INTERFACE_H

#include <mcr_interpolation_planner_moveit/interpolation_planner_context.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/profiler/profiler.h>
#include <fstream>
#include <string>
#include <map>
#include <ros/ros.h>
#include <vector>

/**
 * @brief The MoveIt interface to the Interpolation planner.
 */
namespace interpolation_planner_interface
{
/**
 * @class InterpolationPlannerInterface
 * This class provides an interface between the planner manager and the planning context.
 * It loads and retains the configuration of the planner (such as planning algorithm,
 * planning sample time) and of all the kinematic groups (e.g. joint names, velocity and
 * acceleration bounds). It creates new instances of the planning context based on each
 * new planning request from the planner manager and loaded configurations. These
 * instances are then passed back through the planner manager to the plugin loader
 * application.
 */
class InterpolationPlannerInterface
{
public:
    /**
     * A constructor.
     * @brief Initialize interpolation based planning for a
     *        particular robot model. ROS configuration is read
     *        from the specified NodeHandle.
     * @param pconfig Configurations for the different planners.
     */
    InterpolationPlannerInterface(const robot_model::RobotModelConstPtr &kmodel,
                                  const ros::NodeHandle &nh = ros::NodeHandle("~"));

    /**
     * A destructor.
     */
    virtual ~InterpolationPlannerInterface();

    /**
     * @brief Specify configurations for the planners.
     * @param pconfig Configurations for the different planners.
     */
    void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig);

    /**
     * @brief Get the configurations for the planners that are already loaded.
     * @param pconfig Configurations for the different planners.
     */
    const planning_interface::PlannerConfigurationMap& getPlannerConfigurations() const
    {
        return planner_configs_;
    }

    /**
     * @brief Gets context of the interpolation based planner.
     * @param planning_scene contains planning scane.
     * @param req contains request for the motion to be planned.
     * @param error_code contains status about the initilization of
     *                   planner context.
     * @return      Pointer to interpolation based planning context.
     */
    InterpolationPlannerContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest &req,
        moveit_msgs::MoveItErrorCodes &error_code) const;

    /**
     * @brief Print the status of this node.
     */
    void printStatus();

protected:
    /**
     * @brief Reads planner related configurations from the
     *        parameter server.
     */
    void loadPlannerConfigurations();

    /**
     * @brief Reads group related configurations using robot
     *        kinematic model.
     */
    void loadGroupConfigurations();

    /**
     * ROS node handle.
     */
    ros::NodeHandle nh_;

    /**
     * @brief The kinematic model for which motion plan to be computed.
     */
    robot_model::RobotModelConstPtr kmodel_;

    /**
     * @brief Stores planner related configurations.
     */
    planning_interface::PlannerConfigurationMap planner_configs_;

    /**
     * @brief Stores group related configurations.
     */
    interpolation_planner_interface::GroupSpecificationMap group_configs_;
};
}  // namespace interpolation_planner_interface
#endif  // MCR_INTERPOLATION_PLANNER_MOVEIT_INTERPOLATION_PLANNER_INTERFACE_H
