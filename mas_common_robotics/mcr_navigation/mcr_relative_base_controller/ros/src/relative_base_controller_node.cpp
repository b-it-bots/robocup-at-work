#include <mcr_relative_base_controller/relative_base_controller_node.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>


BaseMotionController::BaseMotionController(ros::NodeHandle &n) : nh_(n)
{
  dynamic_reconfigure_server_.setCallback(boost::bind(&BaseMotionController::dynamicReconfigCallback, this, _1, _2));

  odom_received_ = false;
  done_moving_ = true;
  start_relative_movement_ = false;

  nh_.getParam("world_frame_tf", world_frame_tf_);
  nh_.getParam("base_frame_tf", base_frame_tf_);

  base_velocities_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1 );
  move_done_pub_ = nh_.advertise<std_msgs::String>("event_out", 1);
  
  move_command_sub_ = nh_.subscribe("command", 1, &BaseMotionController::moveBaseCallback, this);
  trigger_sub_ = nh_.subscribe("event_in", 1, &BaseMotionController::triggerCallback, this);
}

BaseMotionController::~BaseMotionController()
{
  trigger_sub_.shutdown();
  move_command_sub_.shutdown();
  base_velocities_pub_.shutdown(); 
}

void BaseMotionController::moveBaseCallback(const geometry_msgs::Twist &relative_move_command)
{
  if (done_moving_)
  {
    x_trans_ = relative_move_command.linear.x;
    y_trans_ = relative_move_command.linear.y;
    z_rot_ = relative_move_command.angular.z;
    
    ROS_DEBUG("Got new move message");
    
    // if both translation and rotation are specified, issue a warning
    if (((x_trans_ != 0.0) || (y_trans_ != 0.0)) && (z_rot_ != 0.0))
    {
      ROS_WARN("Both translation and rotation are set");
      if (rotate_first_)
      {
        ROS_WARN("Rotating first then translating");
      }
      else
      {
        ROS_WARN("Translating first then rotating");
      }
    }
    done_moving_ = false;
    new_command_sent_ = true;
  }
  else 
  {
    ROS_WARN("Base still moving. Ignoring move command");
  }
}

void BaseMotionController::triggerCallback(const std_msgs::String &trigger_command)
{
  if (trigger_command.data == "e_start")
  {
    start_relative_movement_ = true;
    base_odom_sub_ = nh_.subscribe("/odom", 1, &BaseMotionController::odomCallback, this);
    ROS_DEBUG("Got start command");
  }
  else
  {
    ROS_DEBUG("Got stop command");
    start_relative_movement_ = false;
    base_odom_sub_.shutdown();
    x_trans_ = 0.0;
    y_trans_ = 0.0;
    z_rot_ = 0.0;
  }
}

void BaseMotionController::dynamicReconfigCallback(mcr_relative_base_controller::RelativeBaseControllerConfig &config, uint32_t level) 
{
  this->rotate_first_ = config.rotation_before_translation;
  this->x_vel_ = config.x_vel;
  this->y_vel_ = config.y_vel;
  this->theta_vel_ = config.theta_vel;
  this->tolerance_ = config.distance_tolerance;
}
  		
void BaseMotionController::odomCallback(const nav_msgs::Odometry &odom)
{
    double roll, pitch, yaw;      
    tf::Quaternion q;
    x_current_ = odom.pose.pose.position.x;
    y_current_ = odom.pose.pose.position.y;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);  
    theta_current_ = yaw;
    odom_received_ = true; 
} 

void BaseMotionController::run()
{
  while (ros::ok())
  {
    ros::Rate(10).sleep();
    ros::spinOnce();      
    // wait for start trigger and new Twist message 
    if (!start_relative_movement_ || !new_command_sent_)
    {
      continue;
    }
    
    odom_received_ = false;
    if (start_relative_movement_)
    {
      // rotate first then translate
      if (rotate_first_)
      {
        if (z_rot_ != 0.0)
        {
          rotateRelative(z_rot_);      
        }
        if (x_trans_ != 0.0 || y_trans_ != 0.0)
        {
          translateRelative(x_trans_, y_trans_);
        }
      }
      else
      {
        // translate first then rotate
        if (x_trans_ != 0.0 || y_trans_ != 0.0)
        {
          translateRelative(x_trans_, y_trans_);
        }
        if (z_rot_ != 0.0)
        {
          rotateRelative(z_rot_);      
        }
      }
    }
    if (new_command_sent_)
    {
      new_command_sent_ = false;
      x_trans_ = 0.0;
      y_trans_ = 0.0;
      z_rot_ = 0.0;
    }
    done_moving_ = true;
    std_msgs::String move_done_str;
    move_done_str.data = "e_done";
    move_done_pub_.publish(move_done_str);
  }
}

bool BaseMotionController::translateRelative(double x_trans, double y_trans)
{
  geometry_msgs::Twist zero_velocity;
  while (!odom_received_)
  {    
    ros::spinOnce();
    if (!start_relative_movement_)
    {
      base_velocities_pub_.publish(zero_velocity);
      return false;
    }
  }
  odom_received_ = false;

  tf::TransformListener tf_listener;
  tf::StampedTransform robot_transform;
  bool tf_received = false;
  // wait for transform
  // this is required to convert odom values (x and y position) to the base_footprint frame
  while (!tf_received)
  {
    try
    {
      tf_listener.lookupTransform(world_frame_tf_, base_frame_tf_, ros::Time(0), robot_transform);
      tf_received = true;
    }
    catch(tf::TransformException &ex)
    {

    }
  }

  // current orientation of the robot w.r.t starting orientation
  double robot_yaw = tf::getYaw(robot_transform.getRotation());

  // transform point to base frame
  geometry_msgs::Point point;
  point.x = x_current_;
  point.y = y_current_;
  geometry_msgs::Point newPoint = getTransformedPoint(point, robot_yaw);

  // calculate desired x and y positions
  double x_goal = newPoint.x + x_trans;
  double y_goal = newPoint.y + y_trans;

  ROS_DEBUG("Current: x: %f, y: %f ", x_current_, y_current_);
  ROS_DEBUG("Goal: x: %f, y: %f ", x_goal, y_goal);

  // if already at goal positions, set velocity to zero and return
  if (isMovementDone(x_goal, newPoint.x) && isMovementDone(y_goal, newPoint.y))
  {
    base_velocities_pub_.publish(zero_velocity);
    return true;
  }
  
  // while either x or y translation needs to be done
  while (!isMovementDone(x_goal, newPoint.x) || !isMovementDone(y_goal, newPoint.y))
  {      
    geometry_msgs::Twist base_velocity;
    // if x translation is required
    if (!isMovementDone(x_goal, newPoint.x))
    {
      if (x_goal - newPoint.x < 0.0)
      {
        base_velocity.linear.x = -x_vel_;
      }
      else
      {
        base_velocity.linear.x = x_vel_;
      }
    }

    // if y translation is required
    if (!isMovementDone(y_goal, newPoint.y))
    {
      if (y_goal - newPoint.y < 0.0) 
      {
        base_velocity.linear.y = -y_vel_;
      }
      else
      {
        base_velocity.linear.y = y_vel_;
      }
    }

    // set velocity
    base_velocities_pub_.publish(base_velocity);

    // wait till we receive another odom reading
    while (!odom_received_)
    {
      ros::spinOnce();
      if (!start_relative_movement_)
      {        
        base_velocities_pub_.publish(zero_velocity);
        return false;
      }
    }
    odom_received_ = false;

    // transform point to base frame
    point.x = x_current_;
    point.y = y_current_;
    newPoint = getTransformedPoint(point, robot_yaw);
  }
  // set velocity to zero
  base_velocities_pub_.publish(zero_velocity);
  return true;
}

bool BaseMotionController::rotateRelative(double rotation)
{
  geometry_msgs::Twist zero_velocity;
  while (!odom_received_)
  {
    ros::spinOnce();
    if (!start_relative_movement_)
    {      
      base_velocities_pub_.publish(zero_velocity);
      return false;
    }
  }
  odom_received_ = false;

  double theta_goal = (theta_current_ + rotation);
  // if the goal exceeds M_PI, move it to the range -M_PI to 0
  if (theta_goal > M_PI)
  {
    theta_goal = -(2.0 * M_PI - theta_goal);
  }
  // if the goal is less than -M_PI, move it to the range 0 to M_PI
  else if (theta_goal < -M_PI)
  {
    theta_goal = (2.0 * M_PI + theta_goal);
  }

  ROS_DEBUG("Current theta: %f", theta_current_);
  ROS_DEBUG("Goal theta: %f", theta_goal);

  if (isMovementDone(theta_goal, theta_current_))
  {
    base_velocities_pub_.publish(zero_velocity);
    return true;
  }

  while (!isMovementDone(theta_goal, theta_current_))
  {
    geometry_msgs::Twist base_velocity;
    // calculate difference in current and goal position in radians
    double theta_diff = angularDistance(theta_goal, theta_current_);

    if (theta_diff < 0.0)
    {
      base_velocity.angular.z = -theta_vel_;
    }
    else
    {
      base_velocity.angular.z = theta_vel_;
    }

    base_velocities_pub_.publish(base_velocity);

    // wait for another odom reading before checking again
    while (!odom_received_)
    {
      ros::spinOnce();
      if (!start_relative_movement_)
      {
        base_velocities_pub_.publish(zero_velocity);
        return false;
      }
    }
    odom_received_ = false;
  }
  base_velocities_pub_.publish(zero_velocity);
  return true;
}

bool BaseMotionController::isMovementDone(double goal, double current)
{
  // check if current is within 'tolerance_' of goal
  return (fabs(goal - current) < tolerance_);
}
  
// calculate angular distance between the two angles in radians
double BaseMotionController::angularDistance(double angle1, double angle2)
{
  return atan2(sin(angle1 - angle2), cos(angle1 - angle2));
}

geometry_msgs::Point BaseMotionController::getTransformedPoint(const geometry_msgs::Point &point, double robot_yaw)
{
  geometry_msgs::Point newPoint;
  newPoint.x = (point.x * cos(robot_yaw) + point.y * sin(robot_yaw));
  newPoint.y = (point.y * cos(robot_yaw) - point.x * sin(robot_yaw));
  return newPoint;
}
      

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "relative_base_controller");

  ros::NodeHandle n("~");

  ROS_INFO("Ready to move base position");

  BaseMotionController bm(n); 
  bm.run();

  return 0;
}
