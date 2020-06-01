/* ROS node that aims to simulate GoT position
*
* - Subscribe to /got_teensy (geometry_msgs/Pose) published by teensy
*   
* - Subscribes to the following for ground_truth estimates:
*   -- /odom (nav_msgs/Odometry) (odom filtered from robot_localization_ekf)
*   -- /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
*   -- /slam_out_pose (geometry_msgs/PoseStamped) 
* 
* Parameters:
* - got_err_tolerance, default = 0
* - got_err_s, default = 3000   //Higher values trust ground_truth most, lower values are identical to got_teensy
* - got_ground_truth, default="odom" set ground_truth to use odom, amcl or slam
* 
* Publishers
* - got_error - For robot1, other robots subscribe to this topic. Publishes all found position/error pairs.
* - Apply correction and publish to /got_pose_corrected (geometry_msgs/PoseWithCovarianceStamped)
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <bits/stdc++.h>

#include "got_node/AddTwoInts.h"
#include "got_node/GetPosError.h"

#include <fstream>
#include <iostream>

// bool sendErr(got_node::GetPosError::Request  &req,
//          got_node::GetPosError::Response &res)
// {
//   res.err_x = 12.3;
//   res.err_y = 1.0;
//   ROS_INFO("request: x=%f, y=%f", req.x, req.y);
//   ROS_INFO("sending back response: ex:[%f] ey:[%f]", res.err_x, res.err_y);
//   return true;
// }

struct Position { 
	double x, y; 

	Position(float x_, float y_) 
	{ 
		x = x_; 
		y = y_; 
	} 

	bool operator==(const Position& p) const
	{ 
		return x == p.x && y == p.y; 
	}

    bool operator<(const Position &o)  const {
        return x < o.x || (x == o.x && y < o.y);
    }
};


class MyHashFunction { 
public: 

	// We use predfined hash functions of strings 
	// and define our hash function as XOR of the 
	// hash values. 
	size_t operator()(const Position& p) const
	{ 
		return (std::hash<float>()(p.x)) ^ 
			(std::hash<float>()(p.y)); 
	} 
}; 

class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void teensyCallback(const geometry_msgs::Point::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomLocalCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void slamCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  bool sendErr(got_node::GetPosError::Request  &req,
         got_node::GetPosError::Response &res);
  
  ros::NodeHandle nh_;
  
  ros::Subscriber teensy_sub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber odom_local_sub_;
  ros::Subscriber amcl_sub_;
  ros::Subscriber slam_sub_;

  ros::Publisher got_pub_;
  
  unsigned frame_sequence_;


  //Parameters
  double err_s_ = 3000;
  double err_tolerance_ = 0; // 10 cm

  std::string param_ground_truth_ = "amcl";
  bool param_fp_enabled_ = true;


  geometry_msgs::Point amcl_pos_;
  geometry_msgs::Point odom_pos_;
  geometry_msgs::Point slam_pos_;
  geometry_msgs::Point odom_local_pos_;

  std::unordered_map<Position, Position, MyHashFunction> error_map_;
  // Position pos = {0,0};
  // Position err = {0,0};
  ros::ServiceServer service_;
};


TeleopTurtle::TeleopTurtle()
{
  teensy_sub_ = nh_.subscribe<geometry_msgs::Point>("got_teensy", 10, &TeleopTurtle::teensyCallback, this);

  odom_local_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom_local_filtered", 10, &TeleopTurtle::odomLocalCallback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &TeleopTurtle::odomCallback, this);
  amcl_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, &TeleopTurtle::amclCallback, this);
  slam_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 10, &TeleopTurtle::slamCallback, this);

  got_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_pose_corrected", 1);

  service_ = nh_.advertiseService("get_pos_error", &TeleopTurtle::sendErr, this);

  nh_.setParam("got_err_tolerance", err_tolerance_);
  nh_.setParam("got_err_s", err_s_);

  nh_.setParam("got_ground_truth", param_ground_truth_);
  nh_.setParam("got_enable_fp", param_fp_enabled_);

}

void TeleopTurtle::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_pos_.x = msg->pose.pose.position.x;
  odom_pos_.y = msg->pose.pose.position.y;
  odom_pos_.z = 0;
}

void TeleopTurtle::odomLocalCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_local_pos_.x = msg->pose.pose.position.x;
  odom_local_pos_.y = msg->pose.pose.position.y;
  odom_local_pos_.z = 0;
}

void TeleopTurtle::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  amcl_pos_.x = msg->pose.pose.position.x;
  amcl_pos_.y = msg->pose.pose.position.y;
  amcl_pos_.z = 0;
}

void TeleopTurtle::slamCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  slam_pos_.x = msg->pose.position.x;
  slam_pos_.y = msg->pose.position.y;
  slam_pos_.z = 0;
}

bool TeleopTurtle::sendErr(got_node::GetPosError::Request  &req,
         got_node::GetPosError::Response &res)
{
  Position pos = {req.x, req.y};

  if (error_map_.find(pos) != error_map_.end())
  {
      auto it = error_map_.find(pos);

      res.err_x = it->second.x;
      res.err_y = it->second.y;
      //ROS_INFO("Got error X:%f, Y:%f", err_x, err_y);
      ROS_INFO("sending back response: ex:[%f] ey:[%f]", res.err_x, res.err_y);
  }
  else
  {
    ROS_ERROR("Position not found");
    res.err_x = 0;
    res.err_y = 0;
  }

  return true;
}

void TeleopTurtle::teensyCallback(const geometry_msgs::Point::ConstPtr& msg)
{

    nh_.getParam("got_err_tolerance", err_tolerance_); // Error tolerance parameter, defaults to 0
    nh_.getParam("got_err_s", err_s_); // Error tolerance parameter, defaults to 0

    nh_.getParam("got_ground_truth", param_ground_truth_);
    nh_.getParam("got_enable_fp", param_fp_enabled_);


    // Set ground_truth reference to compare with actual GoT position
    double ground_x = 0, ground_y = 0, ground_z = 0;
    double got_x = msg->x;
    double got_y = msg->y; 

    if (param_ground_truth_ == "odom")
    {
      ground_x = odom_pos_.x;
      ground_y = odom_pos_.y;
      ground_z = odom_pos_.z;
      //ROS_INFO("ground_truth set to: odom");
    }
    else if (param_ground_truth_ == "amcl")
    {
      ground_x = amcl_pos_.x;
      ground_y = amcl_pos_.y;
      ground_z = amcl_pos_.z;
      //ROS_INFO("ground_truth set to: amcl");
    }
    else if (param_ground_truth_ == "slam")
    {
      ground_x = slam_pos_.x;
      ground_y = slam_pos_.y;
      ground_z = slam_pos_.z;
      //ROS_INFO("ground_truth set to: slam");
    }

    // Error correction
    double dx = ground_x - msg->x;
    double dy = ground_y - msg->y;

    float got_key_x = ((int)(got_x * 10)) / 10.0;   // Store error for every 10 cm
    float got_key_y = ((int)(got_x * 10)) / 10.0;

    double error_x = 0, error_y = 0;

    if(fabs(dx) > err_tolerance_)
    {
      error_x = -(pow(dx,2)) - (pow(dy,2));
      error_x = dx * exp(error_x / err_s_);
    }

    if(fabs(dy) > err_tolerance_)
    {
      error_y = -(pow(dx,2)) - (pow(dy,2));
      error_y = dy * exp(error_y / err_s_);
    }

    Position pos = {got_key_x, got_key_y};
    if(param_fp_enabled_ && ((error_x != 0) || (error_y != 0)))
    {
      //Position pos = {ground_x, ground_y};
      Position err = {error_x, error_y};
      // Store new position/error pair
        
      std::pair<Position, Position> p(pos, err);
      error_map_.insert(p);
    }

    if (error_map_.find(pos) != error_map_.end())
    {
        auto it = error_map_.find(pos);

        error_x = it->second.x;
        error_y = it->second.y;
        //ROS_INFO("Got error X:%f, Y:%f", err_x, err_y);
        ROS_INFO("Using error: ex:[%f] ey:[%f]", error_x, error_y);
    }
    else
    {
      ROS_ERROR("Position not found");
      error_x = 0;
      error_y = 0;
    }

    // Construct and publish a new GoT message with error correction
    geometry_msgs::PoseWithCovarianceStamped got_pose;
    tf2::Quaternion quat;
    quat.setRPY(0,0,0);

    //got_pose.header.seq = ++ frame_sequence_;
    got_pose.header.stamp = ros::Time::now();
    got_pose.header.frame_id = "odom";
    got_pose.pose.pose.position.x = got_x + error_x;
    got_pose.pose.pose.position.y = got_y + error_y;
    got_pose.pose.pose.position.z = msg->z;
    got_pose.pose.pose.orientation.x = quat.x();
    got_pose.pose.pose.orientation.y = quat.y();
    got_pose.pose.pose.orientation.z = quat.z();
    got_pose.pose.pose.orientation.w = quat.w();

    got_pose.pose.covariance[0] = 0.001;
    got_pose.pose.covariance[7] = 0.001;
    got_pose.pose.covariance[35] = 0.001;

    got_pub_.publish(got_pose);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "got_position_node");
  TeleopTurtle teleop_turtle;

  ros::spin();
}