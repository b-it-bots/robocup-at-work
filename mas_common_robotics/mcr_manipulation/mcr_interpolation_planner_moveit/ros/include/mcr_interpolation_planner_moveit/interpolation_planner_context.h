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
#ifndef MCR_INTERPOLATION_PLANNER_MOVEIT_INTERPOLATION_PLANNER_CONTEXT_H
#define MCR_INTERPOLATION_PLANNER_MOVEIT_INTERPOLATION_PLANNER_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <boost/thread/mutex.hpp>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/profiler/profiler.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <vector>
#include <map>

/**
 * @brief The context for the interpolation based planner.
 */
namespace interpolation_planner_interface
{
struct GroupSpecification;
struct JointSpecification;

/**
 * A structure to store bounds for the joints.
 */
struct JointSpecification
{
    double max_position_;
    double min_position_;
    bool position_bounded_;
    double max_velocity_;
    double min_velocity_;
    bool velocity_bounded_;
    double max_acceleration_;
    double min_acceleration_;
    bool acceleration_bounded_;
};

/**
 * A structure to store group configurations.
 */
struct GroupSpecification
{
    std::vector<std::string> joint_names;
    std::map<std::string, JointSpecification> joint_spec_map;
};

/**
 * A map to store specification for multiple groups.
 */
typedef std::map<std::string, GroupSpecification> GroupSpecificationMap;

/**
 * @class InterpolationPlannerInterface
 * This class provides the implementation of the interpolation based planning algorithm.
 * Given a goal state in the planning request, it generates a trajectory by
 * interpolating from the start state (constructed from the robot model) to the goal state.
 * The plugin loader application uses an instance of the planning context to get a solution
 * for a particular planning request.
 */
class InterpolationPlannerContext : public planning_interface::PlanningContext
{
public:
    /**
     * A constructor.
     * @brief Initialize Interpolation based planning context for a
     *        particular group model.
     * @param name Name of the planning context.
     * @param planner_spec Configurations for the specific planners.
     * @param kmodel Kinematic model for the robot.
     */
    InterpolationPlannerContext(const std::string &context_name,
                                const planning_interface::PlannerConfigurationSettings &planner_spec,
                                const GroupSpecification &group_spec,
                                const robot_model::RobotModelConstPtr &kmodel);

    /**
     * A destructor.
     */
    virtual ~InterpolationPlannerContext();

    /**
     * @see planning_interface::PlanningContext::solve
     */
    virtual bool solve(planning_interface::MotionPlanResponse &res);

    /**
     * @see planning_interface::PlanningContext::solve
     */
    virtual bool solve(planning_interface::MotionPlanDetailedResponse &res);

    /**
     * @see planning_interface::PlanningContext::clear
     */
    virtual void clear();

    /**
     * @see planning_interface::PlanningContext::terminate
     */
    virtual bool terminate();

    /**
     * @brief Set complete initial state of the robot.
     * @param complete_initial_robot_state initial state of the robot.
     */
    void setCompleteInitialState(const robot_state::RobotState &complete_initial_robot_state);

    /**
     * @brief Get robot state.
     */
    const robot_state::RobotState& getCompleteInitialRobotState() const;

    /**
     * @brief Stores goal constraints.
     */
    bool setGoalConstraints(const moveit_msgs::Constraints &goal_constraints,
                            moveit_msgs::MoveItErrorCodes *error);

private:
    /**
     * Copy Ctor.
     */
    InterpolationPlannerContext(const InterpolationPlannerContext &other);

    /**
     * Assignment operator
     */
    InterpolationPlannerContext &operator=(const InterpolationPlannerContext &other);

private:
    /**
     * @brief Get the robot model from robot state.
     */
    const robot_model::RobotModelConstPtr& getRobotModel() const;

    /**
     * @brief Get the amount of time spent computing the last plan.
     */
    double getPlannedTrajectoryTime() const;

    /**
     * @brief Computes trajectory from a start state (constructed from current robot state)
     *        to  a goal state (constructed from the goal constraint). The steps to compute
     *        trajectory are:
     *        1. Prepares start and goal state.
     *        2. Finds longest motion duration to synchronize motion of all the axis.
     *        3. Configures KDL velocity profiler using start and goal state, maximum duration
     *           (found in previous step) alongwith velocity and acceleration bounds.
     *        4. Gets trajectory points(from interpolation) corresponding to each sample
     *           time starting from time equal to zero till the maximum duration.
     *        5. Combines trajectory points to construct complete trajectory.
     */
    bool generateTrajectory();

    /**
     * @brief Finds maximum duration for longest motion of all the axis.
     */
    double searchLongestMotionDuration();

    /**
     * @brief Computes trajectory point using KDL velocity profiler.
     * @param time_from_start sample time for desired trajectory point.
     * @param traj_point contains desired trajectory point.
     * @return Returns false if velocity profiler fails to interpolate trajectory point
                otherwise true.
     */
    bool interpolate(double time_from_start,
                     trajectory_msgs::JointTrajectoryPoint &traj_point);

    /**
     * @brief Loads start configuration from robot initial state.
     */
    virtual bool constructStartState();

    /**
     * @brief Loads goal configuration from goal constraints.
     */
    virtual bool constructGoalState();

    /**
     * @brief Initilizes velocity profiler.
     */
    virtual bool configureVelocityProfiler(double profiler_duration);

protected:
    /**
     * Stores initial state of the robot.
     */
    robot_state::RobotState complete_initial_robot_state_;

    /**
     * Stores robot kinematic model.
     */
    robot_model::RobotModelConstPtr robot_model_;

    /**
     * Stores goal constraints for the path to be planned.
     */
    moveit_msgs::Constraints goal_constraints_msg_;

    /**
     * The total time of the planned trajectory.
     */
    double planned_trajectory_time_;

    /**
     * Stores the planned trajectory.
     */
    trajectory_msgs::JointTrajectory joint_traj_;

    /**
     * Stores the joint names for the specific group.
     */
    std::vector<std::string> group_joint_names_;

    /**
     * Stores joint names specified in the planning request.
     */
    std::vector<std::string> planning_req_joint_names_;


    /**
     * Stores the selected planner id(planner type).
     */
    std::string selected_planner_id_;

    /**
     * Stores the sample time for the planner.
     */
    double planner_sample_time_;

    /**
     * Stores the start state of the group.
     */
    std::map<std::string, double> start_state_map_;

    /**
     * Stores the desired goal state of the group.
     */
    std::map<std::string, double> goal_state_map_;

    /**
     * Store velocity bounds for the group variables.
     */
    std::map<std::string, double> max_velocity_map_;
    std::map<std::string, double> min_velocity_map_;

    /**
     * Store acceleration bounds for the group variables.
     */
    std::map<std::string, double> max_acceleration_map_;
    std::map<std::string, double> min_acceleration_map_;

    /**
    * The interface to the kdl velocity profiler
    * to generate trajectory points.
    */
    std::vector<boost::shared_ptr<KDL::VelocityProfile> > vp_;
};
typedef boost::shared_ptr<InterpolationPlannerContext> InterpolationPlannerContextPtr;
}  // namespace interpolation_planner_interface
#endif  // MCR_INTERPOLATION_PLANNER_MOVEIT_INTERPOLATION_PLANNER_CONTEXT_H
