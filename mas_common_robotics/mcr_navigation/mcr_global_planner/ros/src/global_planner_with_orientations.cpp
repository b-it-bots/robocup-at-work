
/*********************************************************************
 * Copyright·[2015]·<Bonn-Rhein-Sieg·University>
 * Author: Torsten Jandt
 *         torsten.jandt@smail.inf.h-brs.de
 *********************************************************************/


#include <string>
#include <vector>

#include <mcr_global_planner/global_planner_with_orientations.h>
#include <geometry_msgs/PoseArray.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(mcr_global_planner::GlobalPlannerWithOrientations, nav_core::BaseGlobalPlanner)

namespace mcr_global_planner
{
GlobalPlannerWithOrientations::GlobalPlannerWithOrientations() : global_planner::GlobalPlanner::GlobalPlanner(),
                                                                 initialized_(false)
{
}

GlobalPlannerWithOrientations::GlobalPlannerWithOrientations(std::string name, costmap_2d::Costmap2D *costmap,
                                                             std::string frame_id)
    : global_planner::GlobalPlanner::GlobalPlanner(name, costmap, frame_id), initialized_(false)
{
}

void GlobalPlannerWithOrientations::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    GlobalPlanner::initialize(name, costmap_ros);
    initializeThis(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlannerWithOrientations::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id)
{
    GlobalPlanner::initialize(name, costmap, frame_id);
    initializeThis(name, costmap, frame_id);
}

void GlobalPlannerWithOrientations::initializeThis(std::string name,
                                                   costmap_2d::Costmap2D *costmap,
                                                   std::string frame_id)
{
    if (!initialized_)
    {
        initialized_ = true;
        GlobalPlanner::plan_pub_.shutdown();
        ros::NodeHandle private_nh("~/" + name);

        private_nh.param("min_pose_distance", min_pose_distance_, 0.05);
        private_nh.param("max_omni_poses", max_omni_poses_, 20);
        pub_pose_array_ = private_nh.advertise< geometry_msgs::PoseArray >("plan", 1);
    }
}

bool GlobalPlannerWithOrientations::makePlan(const geometry_msgs::PoseStamped &start,
                                             const geometry_msgs::PoseStamped &goal,
                                             std::vector< geometry_msgs::PoseStamped > &plan)
{
    bool result = GlobalPlanner::makePlan(start, goal, plan);
    if (result)
    {
        filterPlan(start, goal, plan);
    }
    return result;
}

bool GlobalPlannerWithOrientations::makePlan(const geometry_msgs::PoseStamped &start,
                                             const geometry_msgs::PoseStamped &goal,
                                             double tolerance,
                                             std::vector< geometry_msgs::PoseStamped > &plan)
{
    bool result = GlobalPlanner::makePlan(start, goal, tolerance, plan);
    if (result)
    {
        filterPlan(start, goal, plan);
    }
    return result;
}

void GlobalPlannerWithOrientations::filterPlan(const geometry_msgs::PoseStamped &start,
                                               const geometry_msgs::PoseStamped &goal,
                                               std::vector< geometry_msgs::PoseStamped > &plan)
{
    const double startRotation = tf::getYaw(start.pose.orientation);
    const double goalRotation = tf::getYaw(goal.pose.orientation);

    // set start and goal rotations inside the plan
    plan[plan.size() - 1].pose.orientation = goal.pose.orientation;
    if (plan.size() < 2)
    {
        return;
    }
    plan[0].pose.orientation = start.pose.orientation;

    double length = 0.0;
    // Remove poses to achieve min pose distance
    for (unsigned int i = 1; i < plan.size() - 1; ++i)
    {
        double d = calcDistance(plan[i], plan[i - 1]);
        if (d < min_pose_distance_)
        {
            plan.erase(plan.begin() + i);
            --i;
        }
        else
        {
            length += d;
        }
    }

    if (plan.size() < max_omni_poses_ * 2 + 2)
    {
        // only omni drive if short distance
        double traveled = 0.0;
        double way1 = modlike(goalRotation - startRotation, M_PI * 2);
        double way2 = modlike(startRotation - goalRotation, M_PI * 2);
        for (unsigned int i = 1; i < plan.size() - 1; ++i) {
            geometry_msgs::Pose *pose = &plan[i].pose;
            traveled += calcDistance(plan[i], plan[i - 1]);
            double weight = traveled / length;
            double rotation = calcOmniRotation(startRotation, way1, way2, weight);
            pose->orientation = tf::createQuaternionMsgFromYaw(rotation);
        }
    }
    else
    {
        double weight, rotation;
        double targetRotation, fromRotation;
        double way1, way2;
        // ************************
        // Start omni
        targetRotation = calcRotation(plan[max_omni_poses_], plan[max_omni_poses_ + 1]);
        way1 = modlike(targetRotation - startRotation, M_PI * 2);
        way2 = modlike(startRotation - targetRotation, M_PI * 2);
        for (unsigned int i = 1; i <= max_omni_poses_; ++i)
        {
            geometry_msgs::Pose *pose = &plan[i].pose;
            weight = static_cast<double>(i) / static_cast<double>(max_omni_poses_);
            rotation = calcOmniRotation(startRotation, way1, way2, weight);
            pose->orientation = tf::createQuaternionMsgFromYaw(rotation);
        }

        // ************************
        // Mid drive
        for (unsigned int i = max_omni_poses_ + 1; i < plan.size() - max_omni_poses_; ++i)
        {
            geometry_msgs::Pose *pose = &plan[i].pose;
            rotation = calcRotation(plan[i], plan[i + 1]);
            pose->orientation = tf::createQuaternionMsgFromYaw(rotation);
        }

        // ************************
        // end omni
        fromRotation = calcRotation(plan[plan.size() - max_omni_poses_ - 1], plan[plan.size() - max_omni_poses_]);
        way1 = modlike(goalRotation - fromRotation, M_PI * 2);
        way2 = modlike(fromRotation - goalRotation, M_PI * 2);
        for (unsigned int i = plan.size() - max_omni_poses_ - 1; i < plan.size() - 1; ++i)
        {
            geometry_msgs::Pose *pose = &plan[i].pose;
            weight = static_cast<double>(i - (plan.size() - max_omni_poses_ - 1)) /
                     static_cast<double>(max_omni_poses_);
            rotation = calcOmniRotation(fromRotation, way1, way2, weight);
            pose->orientation = tf::createQuaternionMsgFromYaw(rotation);
        }
    }

    publishVisualPlan(start, goal, plan);
}

double GlobalPlannerWithOrientations::calcOmniRotation(const double from,
                                                       const double way1,
                                                       const double way2,
                                                       const double weight)
{
    if (way1 < way2)
    {
        return modlike(from + way1 * weight, M_PI * 2);
    }
    else
    {
        return modlike(from - way2 * weight, M_PI * 2);
    }
}
double GlobalPlannerWithOrientations::calcOmniRotation(const double from, const double to, const double weight)
{
    double way1 = modlike(to - from, M_PI * 2);
    double way2 = modlike(from - to, M_PI * 2);
    return calcOmniRotation(from, way1, way2, weight);
}

double GlobalPlannerWithOrientations::calcRotation(const geometry_msgs::PoseStamped &from,
                                                   const geometry_msgs::PoseStamped &to)
{
    return calcRotation(from.pose, to.pose);
}

double GlobalPlannerWithOrientations::calcRotation(const geometry_msgs::Pose &from, const geometry_msgs::Pose &to)
{
    return atan2(to.position.y - from.position.y, to.position.x - from.position.x);
}

double GlobalPlannerWithOrientations::calcDistance(const geometry_msgs::PoseStamped &pose1,
                                                   const geometry_msgs::PoseStamped &pose2)
{
    return calcDistance(pose1.pose, pose2.pose);
}

double GlobalPlannerWithOrientations::calcDistance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2)
{
    double dx = pose2.position.x - pose1.position.x;
    double dy = pose2.position.y - pose1.position.y;
    return sqrt(dx * dx + dy * dy);
}

double GlobalPlannerWithOrientations::modlike(const double number, const double limit)
{
    double ret = number;
    while (ret > limit)
    {
        ret -= limit;
    }
    while (ret < 0.0)
    {
        ret += limit;
    }
    return ret;
}

void GlobalPlannerWithOrientations::publishVisualPlan(const geometry_msgs::PoseStamped &start,
                                                      const geometry_msgs::PoseStamped &goal,
                                                      std::vector< geometry_msgs::PoseStamped > &plan)
{
    geometry_msgs::PoseArray array;

    if (!plan.empty())
    {
        array.header.frame_id = plan[0].header.frame_id;
        array.header.stamp = plan[0].header.stamp;
    }
    for (unsigned int i = 0; i < plan.size(); i++)
    {
        array.poses.push_back(plan[i].pose);
    }

    pub_pose_array_.publish(array);
}
}  // namespace mcr_global_planner


