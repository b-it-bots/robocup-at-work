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

#include <mcr_interpolation_planner_moveit/interpolation_planner_context.h>


interpolation_planner_interface::InterpolationPlannerContext::InterpolationPlannerContext(
    const std::string &context_name,
    const planning_interface::PlannerConfigurationSettings &planner_spec,
    const GroupSpecification &group_spec,
    const robot_model::RobotModelConstPtr &kmodel) :
    planning_interface::PlanningContext(context_name, planner_spec.group),
    complete_initial_robot_state_(kmodel),
    planner_sample_time_(0.0),
    planned_trajectory_time_(0.0)
{
    group_joint_names_ = group_spec.joint_names;

    for (std::size_t i = 0 ; i < group_joint_names_.size() ; ++i)
    {
        JointSpecification j_spec = group_spec.joint_spec_map.at(group_joint_names_[i]);
        max_velocity_map_[group_joint_names_[i]] = j_spec.max_velocity_;
        min_velocity_map_[group_joint_names_[i]] = j_spec.min_velocity_;
        max_acceleration_map_[group_joint_names_[i]] = j_spec.max_acceleration_;
        min_acceleration_map_[group_joint_names_[i]] = j_spec.min_acceleration_;
    }

    selected_planner_id_ = planner_spec.config.at("selected_planner_id");

    planner_sample_time_ = boost::lexical_cast<double>(planner_spec.config.at("planner_minimum_sample_time"));
}

interpolation_planner_interface::InterpolationPlannerContext::~InterpolationPlannerContext()
{
}


bool interpolation_planner_interface::InterpolationPlannerContext::solve(planning_interface::MotionPlanResponse &res)
{
    if (planner_sample_time_ <= 0.0)
    {
        logWarn("Unable to solve the planning problem. Planner sampling time must be greater then zero.");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    if (!generateTrajectory())
    {
        logInform("Unable to solve the planning problem");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    // add info about planned solution
    double ptime = getPlannedTrajectoryTime();
    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));
    res.trajectory_->setRobotTrajectoryMsg(complete_initial_robot_state_, joint_traj_);

    res.planning_time_ = ptime;

    logInform("planning problem solved");
    return true;
}

bool interpolation_planner_interface::InterpolationPlannerContext::solve(
    planning_interface::MotionPlanDetailedResponse &res)
{
    logInform("Unable to solve the planning problem");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
}

void interpolation_planner_interface::InterpolationPlannerContext::clear()
{
    planning_req_joint_names_.clear();

    planned_trajectory_time_ = 0.0;

    start_state_map_.clear();
    goal_state_map_.clear();
}

bool interpolation_planner_interface::InterpolationPlannerContext::terminate()
{
    return true;
}

void interpolation_planner_interface::InterpolationPlannerContext::setCompleteInitialState(
    const robot_state::RobotState &complete_initial_robot_state)
{
    complete_initial_robot_state_ = complete_initial_robot_state;
}

const robot_state::RobotState&
interpolation_planner_interface::InterpolationPlannerContext::getCompleteInitialRobotState() const
{
    return complete_initial_robot_state_;
}

bool interpolation_planner_interface::InterpolationPlannerContext::setGoalConstraints(
    const moveit_msgs::Constraints &goal_constraints,
    moveit_msgs::MoveItErrorCodes *error)
{
    if (goal_constraints.joint_constraints.size() == 0)
    {
        logWarn("%s: No goal constraints specified. There is no planning problem to solve.", name_.c_str());
        if (error)
        {
            error->val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        }

        return false;
    }

    goal_constraints_msg_ = goal_constraints;
    return true;
}

const robot_model::RobotModelConstPtr&
interpolation_planner_interface::InterpolationPlannerContext::getRobotModel() const
{
    return complete_initial_robot_state_.getRobotModel();
}

double interpolation_planner_interface::InterpolationPlannerContext::getPlannedTrajectoryTime() const
{
    return planned_trajectory_time_;
}

bool interpolation_planner_interface::InterpolationPlannerContext::generateTrajectory()
{
    planned_trajectory_time_ = 0.0;

    // Prepare goal state
    if (!constructGoalState())
        return false;

    // Prepare start state
    constructStartState();

    // Configure velocity profiler using planner sample time (assuming
    // that the smallest duration required for any axis motion is equal to
    // planner sample time) and then search duration for longest motion
    // from all axis.
    if (!configureVelocityProfiler(planner_sample_time_))
        return false;
    double max_rescaled_time = searchLongestMotionDuration();
    if (max_rescaled_time == 0.0)
        return false;

    // In case longest motion requires less or equal time to sample time then
    // Reduce the sample time to half to insert one intermediate point.
    if (max_rescaled_time <= planner_sample_time_)
    {
        planner_sample_time_ = max_rescaled_time / 2;
    }

    // Configure velocity profiler with maximum duration to rescaled the motion.
    if (!configureVelocityProfiler(max_rescaled_time))
        return false;

    // Computes trajectory based on sample time and max duration.
    double time_from_start = 0.0;
    while (time_from_start <= max_rescaled_time)
    {
        trajectory_msgs::JointTrajectoryPoint traj_point_temp;
        if (!interpolate(time_from_start, traj_point_temp))
            return false;
        joint_traj_.points.push_back(traj_point_temp);

        time_from_start += planner_sample_time_;
    }


    // If time from start is still less then max scaled time then
    // add last trajectory point at the max_rescaled_time which is goal state.
    if ((time_from_start - planner_sample_time_) < max_rescaled_time)
    {
        trajectory_msgs::JointTrajectoryPoint traj_point_temp;
        if (!interpolate(max_rescaled_time, traj_point_temp))
            return false;
        joint_traj_.points.push_back(traj_point_temp);
    }

    // Store total trajectory time
    planned_trajectory_time_ = max_rescaled_time;

    for (std::size_t i = 0 ; i < planning_req_joint_names_.size() ; ++i)
    {
        joint_traj_.joint_names.push_back(planning_req_joint_names_[i]);
    }

    logInform("Total trajectory time: %f", (planned_trajectory_time_));
    logInform("Interpolated trajectory contains %d points.", (joint_traj_.points.size()));
    return true;
}

double interpolation_planner_interface::InterpolationPlannerContext::searchLongestMotionDuration()
{
    double max_duration = 0;

    // Finding maximum duration from all axis.
    for (std::size_t i = 0 ; i < planning_req_joint_names_.size() ; ++i)
    {
        // Find lengthiest trajectory
        max_duration = std::max(max_duration, vp_[i]->Duration());
    }

    return max_duration;
}

bool interpolation_planner_interface::InterpolationPlannerContext::interpolate(
    double time_from_start,
    trajectory_msgs::JointTrajectoryPoint &traj_point)
{
    for (std::size_t i = 0 ; i < planning_req_joint_names_.size() ; ++i)
    {
        traj_point.positions.push_back(vp_[i]->Pos(time_from_start));
        traj_point.velocities.push_back(vp_[i]->Vel(time_from_start));
        traj_point.accelerations.push_back(vp_[i]->Acc(time_from_start));
    }

    traj_point.time_from_start = ros::Duration(time_from_start);
    return true;
}

bool interpolation_planner_interface::InterpolationPlannerContext::constructStartState()
{
    start_state_map_.clear();

    for (std::size_t i = 0 ; i < planning_req_joint_names_.size() ; ++i)
    {
        const double *current_position =
            complete_initial_robot_state_.getJointPositions(planning_req_joint_names_[i]);
        start_state_map_[planning_req_joint_names_[i]] = *current_position;
    }
}

bool interpolation_planner_interface::InterpolationPlannerContext::constructGoalState()
{
    if (goal_constraints_msg_.joint_constraints.size() == 0) return false;

    planning_req_joint_names_.clear();
    goal_state_map_.clear();

    for (std::size_t i = 0 ; i < goal_constraints_msg_.joint_constraints.size() ; ++i)
    {
        moveit_msgs::JointConstraint joint_constraint = goal_constraints_msg_.joint_constraints[i];
        std::vector<std::string>::iterator it;

        it = find(group_joint_names_.begin(), group_joint_names_.end(), joint_constraint.joint_name);
        if (it != group_joint_names_.end())
        {
            planning_req_joint_names_.push_back(joint_constraint.joint_name);
            goal_state_map_[joint_constraint.joint_name] = joint_constraint.position;
        }
        else
        {
            logInform("Unable to construct goal state. Joint '%s' is not present"
                      "in loaded specification of the group.", joint_constraint.joint_name.c_str());
            return false;
        }
    }
    return true;
}

bool interpolation_planner_interface::InterpolationPlannerContext::configureVelocityProfiler(double profiler_duration)
{
    if (selected_planner_id_.compare("Trapezoidal") == 0)
    {
        vp_.clear();
        vp_.resize(planning_req_joint_names_.size());
        for (std::size_t i = 0 ; i < planning_req_joint_names_.size(); ++i)
        {
            std::string joint_name = planning_req_joint_names_[i];
            vp_[i].reset(new KDL::VelocityProfile_Trap(max_velocity_map_[joint_name],
                         max_acceleration_map_[joint_name]));

            vp_[i]->SetProfileDuration(start_state_map_[joint_name],
                                       goal_state_map_[joint_name],
                                       profiler_duration);
        }
    }
    else
    {
        logError("Implementation fo Planner type '%s' is missing.", selected_planner_id_.c_str());
        return false;
    }
    return true;
}
