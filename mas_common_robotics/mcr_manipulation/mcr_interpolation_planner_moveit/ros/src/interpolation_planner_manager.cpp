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

#include <ros/ros.h>
#include <mcr_interpolation_planner_moveit/interpolation_planner_manager.h>


interpolation_planner_interface::InterpolationPlannerManager::InterpolationPlannerManager():
    planning_interface::PlannerManager(),
    nh_("~")
{
}

interpolation_planner_interface::InterpolationPlannerManager::~InterpolationPlannerManager()
{
}

bool interpolation_planner_interface::InterpolationPlannerManager::initialize(
    const robot_model::RobotModelConstPtr& model, const std::string &ns)
{
    if (!ns.empty())
        nh_ = ros::NodeHandle(ns);
    interpolation_planner_interface_.reset(new InterpolationPlannerInterface(model, nh_));
    return true;
}

bool interpolation_planner_interface::InterpolationPlannerManager::canServiceRequest(
    const moveit_msgs::MotionPlanRequest &req) const
{
    return req.trajectory_constraints.constraints.empty();
}

std::string interpolation_planner_interface::InterpolationPlannerManager::getDescription() const
{
    return "Interpolation";
}

void interpolation_planner_interface::InterpolationPlannerManager::getPlanningAlgorithms(
    std::vector<std::string> &algs) const
{
    const planning_interface::PlannerConfigurationMap &pconfig =
        interpolation_planner_interface_->getPlannerConfigurations();
    planning_interface::PlannerConfigurationMap::const_iterator it = pconfig.begin();

    if (it != pconfig.end())
    {
        planning_interface::PlannerConfigurationSettings pc_temp = it->second;
        int number_of_planners = boost::lexical_cast<int>(pc_temp.config["number_of_planners"]);
        algs.clear();
        algs.reserve(number_of_planners);
        for (int iter = 1; iter <= number_of_planners; iter++)
        {
            std::string planner_type = pc_temp.config["planner_type_" + iter];
            algs.push_back(planner_type);
        }
    }
}

void interpolation_planner_interface::InterpolationPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap &pconfig)
{
    // this call can add a few more configs than we pass in (adds defaults)
    interpolation_planner_interface_->setPlannerConfigurations(pconfig);

    // so we read the configs instead of just setting pconfig

    PlannerManager::setPlannerConfigurations(interpolation_planner_interface_->getPlannerConfigurations());
}

planning_interface::PlanningContextPtr
interpolation_planner_interface::InterpolationPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest &req,
    moveit_msgs::MoveItErrorCodes &error_code) const
{
    return interpolation_planner_interface_->getPlanningContext(planning_scene, req, error_code);
}
