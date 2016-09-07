/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <algorithm>
#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#include <mcr_perception_msgs/PersonList.h>
#include <mcr_perception_msgs/Person.h>

#include <mcr_algorithms/geometry/conversions.h>

#include "mcr_leg_detection/PositionMeasurement.h"
#include "mcr_leg_detection/LegDetectionConfig.h"
#include "mcr_leg_detection/laser_processor.h"
#include "mcr_leg_detection/calc_leg_features.h"
#include "mcr_leg_detection/tracker_kalman.h"
#include "mcr_leg_detection/state_pos_vel.h"
#include "mcr_leg_detection/rgb.h"


using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

bool is_detection_enabled = false;

static const double no_observation_timeout_s = 0.5;
static const double max_second_leg_age_s = 2.0;
static const double max_track_jump_m = 1.0;
static const double max_meas_jump_m = 0.75;  // 1.0
static const double leg_pair_separation_m = 1.0;
static const string fixed_frame = "/base_link";

bool start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    is_detection_enabled = true;
    ROS_INFO("leg detector ENABLED");
    return true;
}

bool stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    is_detection_enabled = false;
    ROS_INFO("leg detector DISABLED");
    return true;
}

class SavedFeature
{
public:
    static int nextid;
    TransformListener& tfl_;

    BFL::StatePosVel sys_sigma_;
    TrackerKalman filter_;

    string id_;
    string object_id;
    ros::Time time_;
    ros::Time meas_time_;

    Stamped<Point> position_;
    float dist_to_person_;

    // one leg tracker
    SavedFeature(Stamped<Point> loc, TransformListener& tfl)
        : tfl_(tfl),
          sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)),
          filter_("tracker_name", sys_sigma_)
    {
        char id[100];
        snprintf(id, 100, "legtrack%d", nextid++);
        id_ = std::string(id);

        object_id = "";
        time_ = loc.stamp_;
        meas_time_ = loc.stamp_;

        try
        {
            tfl_.transformPoint(fixed_frame, loc, loc);
        }
        catch (...)
        {
            ROS_WARN("TF exception spot 6.");
        }
        StampedTransform pose(tf::Transform(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
        tfl_.setTransform(pose);

        StatePosVel prior_sigma(Vector3(0.1, 0.1, 0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
        filter_.initialize(loc, prior_sigma, time_.toSec());

        StatePosVel est;
        filter_.getEstimate(est);

        updatePosition();
    }

    void propagate(ros::Time time)
    {
        time_ = time;

        filter_.updatePrediction(time.toSec());

        updatePosition();
    }

    void update(Stamped<Point> loc)
    {
        StampedTransform pose(tf::Transform(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
        tfl_.setTransform(pose);

        meas_time_ = loc.stamp_;
        time_ = meas_time_;

        SymmetricMatrix cov(3);
        cov = 0.0;
        cov(1, 1) = 0.0025;
        cov(2, 2) = 0.0025;
        cov(3, 3) = 0.0025;

        filter_.updateCorrection(loc, cov);

        updatePosition();
    }

    double getLifetime()
    {
        return filter_.getLifetime();
    }

private:
    void updatePosition()
    {
        StatePosVel est;
        filter_.getEstimate(est);

        position_[0] = est.pos_[0];
        position_[1] = est.pos_[1];
        position_[2] = est.pos_[2];
        position_.stamp_ = time_;
        position_.frame_id_ = fixed_frame;
    }
};

int SavedFeature::nextid = 0;

class MatchedFeature
{
public:
    SampleSet* candidate_;
    SavedFeature* closest_;
    float distance_;

    MatchedFeature(SampleSet* candidate, SavedFeature* closest, float distance)
        : candidate_(candidate),
          closest_(closest),
          distance_(distance)
    {
    }

    inline bool operator<(const MatchedFeature& b) const
    {
        return (distance_ < b.distance_);
    }
};

int g_argc;
char** g_argv;

// actual leg detector node
class LegDetection
{
public:
    NodeHandle nh_;

    TransformListener tfl_;
    ScanMask mask_;
    int mask_count_;
    CvRTrees forest;
    float connected_thresh_;
    int feat_count_;
    char save_[100];
    list<SavedFeature*> saved_features_;
    boost::mutex saved_mutex_;
    int feature_id_;

    ros::Publisher pub_combined_legs_;
    ros::Publisher pub_legs_;
    ros::Publisher pub_visualization_marker_;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

    dynamic_reconfigure::Server<mcr_leg_detection::LegDetectionConfig> dynamic_reconfig_server_;
    mcr_leg_detection::LegDetectionConfig dyn_recfg_config_;

    LegDetection(ros::NodeHandle nh)
        : nh_(nh),
          mask_count_(0),
          connected_thresh_(0.06),
          feat_count_(0),
          laser_sub_(nh_, "scan", 10),
          laser_notifier_(laser_sub_, tfl_, fixed_frame, 10)
    {
        if (g_argc > 1)
        {
            forest.load(g_argv[1]);
            feat_count_ = forest.get_active_var_mask()->cols;
            printf("Loaded forest with %d features: %s\n", feat_count_, g_argv[1]);
        }
        else
        {
            printf("Please provide a trained random forests classifier as an input.\n");
            shutdown();
        }

        // advertise topic
        pub_legs_ = nh_.advertise < mcr_perception_msgs::PersonList > ("leg_positions", 1);
        pub_visualization_marker_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

        laser_notifier_.registerCallback(boost::bind(&LegDetection::laserCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));

        dynamic_reconfig_server_.setCallback(boost::bind(&LegDetection::dynamic_reconfig_callback, this, _1, _2));

        feature_id_ = 0;
    }

    void publishVisualizationMarker(const mcr_perception_msgs::PersonList &person_list)
    {
        visualization_msgs::MarkerArray marker_array;

        // convert person msgs into markers for rviz visualization
        for (unsigned int i = 0, j = 0; i < person_list.persons.size(); ++i)
        {
            visualization_msgs::Marker marker_shape;

            marker_shape.header.frame_id = person_list.persons[i].pose.header.frame_id;
            marker_shape.header.stamp = ros::Time::now();
            marker_shape.ns = "leg detections";
            marker_shape.action = visualization_msgs::Marker::ADD;

            marker_shape.id = ++j;

            marker_shape.type = visualization_msgs::Marker::CUBE;

            marker_shape.pose.position = person_list.persons[i].pose.pose.position;
            marker_shape.pose.orientation = person_list.persons[i].pose.pose.orientation;

            marker_shape.scale.x = 0.1;
            marker_shape.scale.y = 0.1;
            marker_shape.scale.z = 0.1;
            marker_shape.color.r = 0.0;
            marker_shape.color.g = 1.0;
            marker_shape.color.b = 0.0;
            marker_shape.color.a = 0.5;;

            marker_shape.lifetime = ros::Duration(1);

            marker_array.markers.push_back(marker_shape);
        }

        pub_visualization_marker_.publish(marker_array);
    }

    void dynamic_reconfig_callback(mcr_leg_detection::LegDetectionConfig &config, uint32_t level)
    {
        dyn_recfg_config_ = config;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        //if not enabled, no processing
        if (!is_detection_enabled) return;

        ScanProcessor processor(*scan, mask_);

        processor.splitConnected(connected_thresh_);
        processor.removeLessThan(5);

        CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);

        // if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
        ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);
        list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
        while (sf_iter != saved_features_.end())
        {
            if ((*sf_iter)->meas_time_ < purge)
            {
                delete(*sf_iter);
                saved_features_.erase(sf_iter++);
            }
            else ++sf_iter;
        }

        // System update of trackers, and copy updated ones in propagate list
        list<SavedFeature*> propagated;
        for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin(); sf_iter != saved_features_.end(); sf_iter++)
        {
            (*sf_iter)->propagate(scan->header.stamp);
            propagated.push_back(*sf_iter);
        }

        // Detection step: build up the set of "candidate" clusters
        list<SampleSet*> candidates;
        for (list<SampleSet*>::iterator i = processor.getClusters().begin(); i != processor.getClusters().end(); i++)
        {
            vector<float> f = calcLegFeatures(*i, *scan);

            for (int k = 0; k < feat_count_; k++)
                tmp_mat->data.fl[k] = (float)(f[k]);

            if (forest.predict(tmp_mat) > 0)
            {
                candidates.push_back(*i);
            }
        }

        // For each candidate, find the closest tracker (within threshold) and add to the match list
        // If no tracker is found, start a new one
        multiset<MatchedFeature> matches;
        for (list<SampleSet*>::iterator cf_iter = candidates.begin(); cf_iter != candidates.end(); cf_iter++)
        {
            Stamped < Point > loc((*cf_iter)->center(), scan->header.stamp, scan->header.frame_id);
            try
            {
                tfl_.transformPoint(fixed_frame, loc, loc);
            }
            catch (...)
            {
                ROS_WARN("TF exception spot 3.");
            }

            list<SavedFeature*>::iterator closest = propagated.end();
            float closest_dist = max_track_jump_m;

            for (list<SavedFeature*>::iterator pf_iter = propagated.begin(); pf_iter != propagated.end(); pf_iter++)
            {
                // find the closest distance between candidate and trackers
                float dist = loc.distance((*pf_iter)->position_);
                if (dist < closest_dist)
                {
                    closest = pf_iter;
                    closest_dist = dist;
                }
            }
            // Nothing close to it, start a new track
            if (closest == propagated.end())
            {
                list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
            }
            // Add the candidate, the tracker and the distance to a match list
            else matches.insert(MatchedFeature(*cf_iter, *closest, closest_dist));
        }

        // loop through _sorted_ matches list
        // find the match with the shortest distance for each tracker
        while (matches.size() > 0)
        {
            multiset<MatchedFeature>::iterator matched_iter = matches.begin();
            bool found = false;
            list<SavedFeature*>::iterator pf_iter = propagated.begin();
            while (pf_iter != propagated.end())
            {
                // update the tracker with this candidate
                if (matched_iter->closest_ == *pf_iter)
                {
                    // Transform candidate to fixed frame
                    Stamped < Point > loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
                    try
                    {
                        tfl_.transformPoint(fixed_frame, loc, loc);
                    }
                    catch (...)
                    {
                        ROS_WARN("TF exception spot 4.");
                    }

                    // Update the tracker with the candidate location
                    matched_iter->closest_->update(loc);

                    // remove this match and
                    matches.erase(matched_iter);
                    propagated.erase(pf_iter++);
                    found = true;
                    break;
                }
                // still looking for the tracker to update
                else
                {
                    pf_iter++;
                }
            }

            // didn't find tracker to update, because it was deleted above
            // try to assign the candidate to another tracker
            if (!found)
            {
                Stamped < Point > loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
                try
                {
                    tfl_.transformPoint(fixed_frame, loc, loc);
                }
                catch (...)
                {
                    ROS_WARN("TF exception spot 5.");
                }

                list<SavedFeature*>::iterator closest = propagated.end();
                float closest_dist = max_track_jump_m;

                for (list<SavedFeature*>::iterator remain_iter = propagated.begin(); remain_iter != propagated.end(); remain_iter++)
                {
                    float dist = loc.distance((*remain_iter)->position_);
                    if (dist < closest_dist)
                    {
                        closest = remain_iter;
                        closest_dist = dist;
                    }
                }

                // no tracker is within a threshold of this candidate
                // so create a new tracker for this candidate
                if (closest == propagated.end()) list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(),
                            new SavedFeature(loc, tfl_));
                else matches.insert(MatchedFeature(matched_iter->candidate_, *closest, closest_dist));
                matches.erase(matched_iter);
            }
        }

        cvReleaseMat(&tmp_mat);
        tmp_mat = 0;

        /*
         * From here it's Fred's extension
         */

        int i = 0;
        mcr_perception_msgs::PersonList person_list;
        double distance = 0, angle = 0;
        geometry_msgs::Quaternion quat;

        for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin(); sf_iter != saved_features_.end(); sf_iter++, i++)
        {
            // reliability
            StatePosVel est;
            (*sf_iter)->filter_.getEstimate(est);

            mcr_perception_msgs::Person person;

            person_list.header.frame_id = person.header.frame_id = person.pose.header.frame_id = fixed_frame;
            person_list.header.stamp = person.header.stamp = person.pose.header.stamp = scan->header.stamp;
            person.id = 0;
            person.is_tracked = false;
            person.pose.pose.position.x = est.pos_[0];
            person.pose.pose.position.y = est.pos_[1];
            Conversions::cartesian2polar2D(est.pos_[0], est.pos_[1], distance, angle);
            quat = tf::createQuaternionMsgFromYaw(angle);
            person.pose.pose.orientation = quat;
            person.probability = 0.0;

            person_list.persons.push_back(person);
        }

        //publish detected legs
        pub_legs_.publish(person_list);

        if (dyn_recfg_config_.publish_visualization_marker)
            publishVisualizationMarker(person_list);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leg_detection");
    g_argc = argc;
    g_argv = argv;
    ros::NodeHandle nh("~");
    LegDetection ld(nh);

    ros::ServiceServer srv_start_detection = nh.advertiseService("start", start);
    ros::ServiceServer srv_stop_detection = nh.advertiseService("stop", stop);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        ros::spinOnce();

        if (!is_detection_enabled) loop_rate.sleep();
    }

    return 0;
}
