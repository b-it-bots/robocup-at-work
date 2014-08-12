#ifndef RELATIVE_BASE_CONTROLLER_H_
#define RELATIVE_BASE_CONTROLLER_H_

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/String.h>
#include <mcr_relative_base_controller/RelativeBaseControllerConfig.h>

/**
 * Moves the base relative to its current position w.r.t the base_link
 *
 * Usage:
 * Publish "e_start" to topic ~/event_in to enable node
 * Publish Twist message to topic ~/command to start relative movement
 * Publish "e_stop" to topic ~/event_in to disable node
 * The node publishes "e_done" to the topic ~/event_out when the relative movement is complete
 *
 * Behaviour:
 * If both translation and rotation are set in the Twist message, order of operation will be decided 
 * by the "rotation_before_translation" parameter (dynamic reconfigure)
 *
 * If a Twist message is sent while the base is moving, it will be ignored
 *
 * If a Twist message is sent while the node is disabled, it will be ignored
 */

class BaseMotionController
{

   public:

    /**
     * Constructor - initializes publishers, subscribers and flags
     */
    BaseMotionController(ros::NodeHandle &n);
    
    /**
     * Destructor - shuts down subscribers
     */
    virtual ~BaseMotionController();
 
    /**
     * Callback for ~/command topic
     * Sets relative movement variables (x_trans_, y_trans_, z_rot_)
     * Sets new_command_sent_ flag to true
     */
    void moveBaseCallback(const geometry_msgs::Twist &relative_move_command);
    /**
     * Callback from ~/event_in topic
     * if event is e_start, sets start_relative_movement_ flag to true, false otherwise
     */
    void triggerCallback(const std_msgs::String &trigger_command);

    /**
     * Callback for dynamic reconfigure
     */
    void dynamicReconfigCallback(mcr_relative_base_controller::RelativeBaseControllerConfig &config, uint32_t level);
        
    /**
     * Callback for /odom topic
     * sets values of x_current_, y_current_ and theta_current_
     */
    void odomCallback(const nav_msgs::Odometry &Odom);

    /**
     * Main running loop
     * Continuously waits for start trigger and relative movement command
     * calls rotateRelative and translateRelative when a command is received
     */
    void run();

  private:

    /**
     * performs relative translation
     */
    bool translateRelative(double x_trans, double y_trans);

    /**
     * performs relative rotation
     */
    bool rotateRelative(double rotation);

    /**
     * checks whether current is within 'tolerance_' of goal
     */
    bool isMovementDone(double goal, double current);
    
    /*
     * calculates angular distance between the two angles in radians
     */
    double angularDistance(double angle1, double angle2);

    /**
     * Transforms point from world frame to base frame based on current robot_yaw
     */
    geometry_msgs::Point getTransformedPoint(const geometry_msgs::Point &point, double robot_yaw);

    /**
     * decides whether rotation is done before translation
     * when both rotation and translation are specified
     */
    bool rotate_first_;

    double x_vel_; /// velocity of translation along x-axis in m/s
    double y_vel_; /// velocity of translation along y-axis in m/s
    double theta_vel_; /// velocity of rotation in rad/s

    double x_current_; /// current x coordinate of base
    double y_current_; /// current y coordinate of base
    double theta_current_; /// current yaw of base

    double x_trans_; /// desired x translation
    double y_trans_; /// desired y translation
    double z_rot_; /// desired z rotation

    std::string world_frame_tf_; /// either odom or map tf
    std::string base_frame_tf_; /// base link tf

    /// set true if odom readings have been recieved
    bool odom_received_; 
    
    /// set true if current relative movement is complete
    bool done_moving_;
    
    /// set true if "start" trigger has been received
    bool start_relative_movement_;

    /// set true if new move relative Twist message has been received
    bool new_command_sent_;

    /// determines how close to the target the base should get
    double tolerance_;

    ros::Publisher base_velocities_pub_; /// base velocity publisher
    ros::Subscriber move_command_sub_; /// subscriber for relative movement command
    ros::Subscriber trigger_sub_; /// subscriber for start/stop trigger 
    ros::Subscriber base_odom_sub_; /// odom subscriber

    ros::Publisher move_done_pub_; /// publisher for event_out

    ros::NodeHandle nh_; /// node handle

    dynamic_reconfigure::Server<mcr_relative_base_controller::RelativeBaseControllerConfig> dynamic_reconfigure_server_; /// dynamic reconfigure server
};

#endif
