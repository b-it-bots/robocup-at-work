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
#ifndef MCR_INTERPOLATION_PLANNER_MOVEIT_INTERPOLATION_PLANNER_MANAGER_H
#define MCR_INTERPOLATION_PLANNER_MOVEIT_INTERPOLATION_PLANNER_MANAGER_H

#include <ros/ros.h>
#include <mcr_interpolation_planner_moveit/interpolation_planner_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <class_loader/class_loader.h>
#include <boost/thread/mutex.hpp>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <string>
#include <vector>


/**
 * @brief The interpolation based planner manager.
 */
namespace interpolation_planner_interface
{
/**
 * @class InterpolationPlannerManager
 * This class is the entry point to the interpolation based planner plugin
 * for an application that loads the plugin. It manages the communication
 * between plugin loader application and the interpolation planning interface
 * class. The communication involves initialization and configuration of the
 * planner interface and getting an instance of the planning context from the
 * planner interface for each new planning request. The instance of the planning
 * context is then utilized to get a solution for each particular request.
 */
class InterpolationPlannerManager : public planning_interface::PlannerManager
{
public:
    /**
     * A constructor.
     */
    InterpolationPlannerManager();

    /**
     * A destructor.
     */
    virtual ~InterpolationPlannerManager();

    /**
     * @brief Initilized planner interface.
     * @param model kinematic model of the robot.
     * @param ns name space for the interface.
     */
    virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string &ns);

    /**
     * @brief Determine whether this plugin instance is able to represent this planning request.
     */
    virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const;

    /**
     * Get a short string that identifies the planning interface.
     */
    virtual std::string getDescription() const;

    /**
     * @brief Get the names of the known planning algorithms (values that can be filled
     *        as planner_id in the planning request).
     */
    virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const;

    /**
     * @brief Specify the settings to be used for specific algorithms.
     */
    virtual void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig);

    /**
     * @brief Construct a planning context given the current scene and a planning request.
     *        If a problem is encountered, error code is set and empty ptr is returned.
     *        The returned motion planner context is clean -- the motion planner will start
     *        from scratch every time a context is constructed.
     * @param planning_scene   A const planning scene to use for planning.
     * @param req The representation of the planning request.
     * @param error_code  This is where the error is set if constructing the planning context fails.
     */
    virtual planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest &req,
        moveit_msgs::MoveItErrorCodes &error_code) const;
private:
    /**
     * Copy Ctor.
     */
    InterpolationPlannerManager(const InterpolationPlannerManager &other);

    /**
     * Assignment operator
     */
    InterpolationPlannerManager &operator=(const InterpolationPlannerManager &other);


private:
    /**
     * The ROS node handle.
     */
    ros::NodeHandle nh_;

    /**
    * The interface to the interpolation planner interface to get planner context.
    */
    boost::scoped_ptr<InterpolationPlannerInterface> interpolation_planner_interface_;
};

}  // namespace interpolation_planner_interface
CLASS_LOADER_REGISTER_CLASS(interpolation_planner_interface::InterpolationPlannerManager,
                            planning_interface::PlannerManager);
#endif  // MCR_INTERPOLATION_PLANNER_MOVEIT_INTERPOLATION_PLANNER_MANAGER_H
