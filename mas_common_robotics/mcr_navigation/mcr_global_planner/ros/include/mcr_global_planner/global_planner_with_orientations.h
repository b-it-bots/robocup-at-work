#ifndef MCR_GLOBAL_PLANNER_GLOBAL_PLANNER_WITH_ORIENTATIONS_H
#define MCR_GLOBAL_PLANNER_GLOBAL_PLANNER_WITH_ORIENTATIONS_H

/********************************************************************
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * Author: Torsten Jandt
 *         torsten.jandt@smail.inf.h-brs.de
 *********************************************************************/

#include <string>
#include <vector>
#include <global_planner/planner_core.h>

namespace mcr_global_planner
{
class GlobalPlannerWithOrientations : public global_planner::GlobalPlanner
{
    private:
        // Minimun distance between two poses.
        double min_pose_distance_;
        // Maximum amount of poses that should be driven in onmi mode only
        int max_omni_poses_;
        // Publisher used to publish the pose-array
        ros::Publisher pub_pose_array_;
        bool initialized_;

        /*
         * Filters the plan by removing poses and adds orientations to remaining ones.
         */
        void filterPlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                        std::vector< geometry_msgs::PoseStamped > &plan);
        /*
         * Initialization for this wrapper class.
         */
        void initializeThis(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id);

        /*
         * Used to publish the plan with orientations as PoseArray
         */
        void publishVisualPlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                               std::vector< geometry_msgs::PoseStamped > &plan);

        /*
         * Calculates the distance between two poses.
         */
        double calcDistance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);
        double calcDistance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);

        /*
         * Calculates the rotation from pose to another
         */
        double calcRotation(const geometry_msgs::Pose &from, const geometry_msgs::Pose &to);
        double calcRotation(const geometry_msgs::PoseStamped &from, const geometry_msgs::PoseStamped &to);

        /*
         * Calculates the rotation using the right rotation direction with precalced values.
         */
        double calcOmniRotation(const double from, const double way1, const double way2, const double weight);

        /*
         * Calculates the rotation using the right rotation direction.
         */
        double calcOmniRotation(const double from, const double to, const double weight);

        /*
         * mod like helper method
         */
        double modlike(const double number, const double limit);

    public:
        GlobalPlannerWithOrientations();
        GlobalPlannerWithOrientations(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id);

        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id);

        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                      std::vector< geometry_msgs::PoseStamped > &plan);
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, double tolerance,
                      std::vector< geometry_msgs::PoseStamped > &plan);
};
}  // namespace mcr_global_planner

#endif  // MCR_GLOBAL_PLANNER_GLOBAL_PLANNER_WITH_ORIENTATIONS_H


