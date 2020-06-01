/* ROS node that aims to simulate GoT position
*
* - Define 5 beacon positions
* - Gazebo plugin publishes e.g. robot1/ground_truth as nav_msgs/Odometry
 - Calculate distance from each beacon to robot
 - Add noise by reportinf e.g once beacon distance * 1.1
 - Get x,y,z position estimate
 - Publish position estimate to got_pose as geometry:msgs/PoseWithCovarianceStamped
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
//#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <bits/stdc++.h>

#include "got_node/GetPosError.h"

#include <fstream>
#include <iostream>

#define NUM_BEACONS 5
//#define WRITE_TO_FILE

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
  void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomFilteredCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomFilteredLocalCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void slamCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  bool sendErr(got_node::GetPosError::Request  &req,
         got_node::GetPosError::Response &res);
  
  ros::NodeHandle nh_;

  ros::Publisher got_pub_;
  ros::Publisher got_corrected_pub_;
  
  ros::Subscriber odom_sub_;
  ros::Subscriber  odom_filtered_sub_;
  ros::Subscriber  odom_filtered_local_sub_;
  ros::Subscriber slam_sub_;
  
  unsigned frame_sequence_;

  //double got_x_=0, got_y_=0, got_z_=0;

  const double beacon_locations_[NUM_BEACONS][3] = { {-2,4,5}, {-2,-4,5}, {2,0,5}, {6,4,5}, {6,-4,5}};

  double err_s_ = 400;
  double err_tolerance_ = 0; // 10 cm

  geometry_msgs::Point odom_filtered_;
  geometry_msgs::Point odom_filtered_local_;
  geometry_msgs::Point slam_pos_;

  std::unordered_map<Position, Position, MyHashFunction> error_map_;

  ros::ServiceServer service_;
};


TeleopTurtle::TeleopTurtle()
{
  got_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_pose", 1);
  got_corrected_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_pose_corrected", 1);

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("ground_truth", 10, &TeleopTurtle::groundTruthCallback, this);
  odom_filtered_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom_global_got_filtered", 10, &TeleopTurtle::odomFilteredCallback, this);
  odom_filtered_local_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom_local_filtered", 10, &TeleopTurtle::odomFilteredLocalCallback, this);
  slam_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 10, &TeleopTurtle::slamCallback, this);

  service_ = nh_.advertiseService("get_pos_error", &TeleopTurtle::sendErr, this);

  nh_.setParam("got_err_tolerance", err_tolerance_);
  nh_.setParam("got_err_s", err_s_);
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

void TeleopTurtle::odomFilteredCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_filtered_.x = msg->pose.pose.position.x;
  odom_filtered_.y = msg->pose.pose.position.y;
  odom_filtered_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::odomFilteredLocalCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_filtered_local_.x = msg->pose.pose.position.x;
  odom_filtered_local_.y = msg->pose.pose.position.y;
  odom_filtered_local_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::slamCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  slam_pos_.x = msg->pose.position.x;
  slam_pos_.y = msg->pose.position.y;
  slam_pos_.z = 0;
}

void TeleopTurtle::groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double ground_x = msg->pose.pose.position.x;
  double ground_y = msg->pose.pose.position.y;
  double ground_z = msg->pose.pose.position.y;

  double got_x_ = 0, got_y_ = 0, got_z_ = 0;

  //Calculate distance from each beacon to the robot
  for(uint8_t index = 0; index < NUM_BEACONS; index++)
  {
      // Actual distance in meters from beacon to robot
      double distance = pow(beacon_locations_[index][0] - ground_x, 2);
      distance += pow(beacon_locations_[index][1] - ground_y, 2);
      distance += pow(beacon_locations_[index][2] - ground_z, 2);
      distance = sqrt(distance);

      // Insert distance error for lowerleft beacon
      if (index == 0)
      {
        distance *= 1.1;
      }

      if(distance>10 || distance<1)
          continue;

      double dp=pow(got_x_- beacon_locations_[index][0], 2);
      dp+=pow(got_y_- beacon_locations_[index][1], 2);
      dp+=pow(got_z_- beacon_locations_[index][2], 2);
      dp=sqrt(dp);

      // Normalizing factor
      double lambda = 1 - dp / distance;
      //ROS_INFO("Lambda: %f", lambda);

      // Estimated Cartesian coordinates
      got_x_=got_x_/(1-lambda)-beacon_locations_[index][0]*lambda/(1-lambda);
      got_y_=got_y_/(1-lambda)-beacon_locations_[index][1]*lambda/(1-lambda);
      got_z_=got_z_/(1-lambda)-beacon_locations_[index][2]*lambda/(1-lambda);
      //ROS_INFO("Position: %d, %f, %f, %f;", index, x_est, y_est, z_est);
  }

    nh_.getParam("got_err_tolerance", err_tolerance_); // Error tolerance parameter, defaults to 0
    nh_.getParam("got_err_s", err_s_); // Error tolerance parameter, defaults to 0

    // Error correction
    double dx = ground_x - got_x_;
    double dy = ground_y - got_y_;

    float got_key_x = ((int)(got_x_ * 10)) / 10.0;
    float got_key_y = ((int)(got_y_ * 10)) / 10.0;
    //ROS_INFO("Key_x: [%f] key_y: [%f]",got_key_x, got_key_y);

    //double error_x = 0, error_y = 0;
    float error_x = 0, error_y = 0;

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
    if((error_x != 0) || (error_y != 0))
    {
      //Position pos = {ground_x, ground_y};
      Position err = {error_x, error_y};
      // Store new position/error pair
        
      std::pair<Position, Position> p(pos, err);
      error_map_.insert(p);
    }

    // if (error_map_.find(pos) != error_map_.end())
    // {
    //     auto it = error_map_.find(pos);

    //     error_x = it->second.x;
    //     error_y = it->second.y;
    //     ROS_INFO("Got error X:%f, Y:%f", error_x, error_y);
    //     //ROS_INFO("sending back response: ex:[%f] ey:[%f]", res.err_x, res.err_y);
    // }
    // else
    // {
    //   ROS_ERROR("Position not found");
    //   error_x = 0;
    //   error_y = 0;
    // }



    #ifdef WRITE_TO_FILE
      std::ofstream myfile;
      myfile.open ("/home/isircu/err_log.csv", std::ios::out | std::ios::app);
      //myfile << dx << ',' << dy << ',' << ground_x - odom_filtered_local_.x << ',' << ground_y - odom_filtered_local_.y << ground_x - odom_filtered_.x << ',' << ground_y - odom_filtered_.y << '\n';

      // myfile << ground_x - odom_filtered_local_.x << ','
      //        << ground_y - odom_filtered_local_.y << ','
      //        << ground_x - odom_filtered_.x << ','
      //        << ground_y - odom_filtered_.y << ','
      //        << dx + error_x << ','
      //        << dy + error_y << ','
      //        << dx << ','
      //        << dy << '\n';
      
      myfile << ground_x -slam_pos_.x << ','
             << ground_y -slam_pos_.y << ','
             << slam_pos_.x - odom_filtered_local_.x << ','
             << slam_pos_.y - odom_filtered_local_.y << ','
             << slam_pos_.x - odom_filtered_.x << ','
             << slam_pos_.y - odom_filtered_.y << ','
             << slam_pos_.x - got_x_ << ','
             << slam_pos_.y - got_y_ << '\n';

      myfile.close();
    #endif


  geometry_msgs::PoseWithCovarianceStamped current_pose;
  tf2::Quaternion quat;
  quat.setRPY(0,0,0);

  current_pose.header.seq = ++ frame_sequence_;
  current_pose.header.stamp = ros::Time::now();
  current_pose.header.frame_id = "odom";

  // current_pose.pose.pose.position.x = msg->pose.pose.position.x;
  // current_pose.pose.pose.position.y = msg->pose.pose.position.y;
  // current_pose.pose.pose.position.z = msg->pose.pose.position.z;

  current_pose.pose.pose.position.x = got_x_;
  current_pose.pose.pose.position.y = got_y_;
  current_pose.pose.pose.position.z = got_z_;

  current_pose.pose.pose.orientation.x = quat.x();
  current_pose.pose.pose.orientation.y = quat.y();
  current_pose.pose.pose.orientation.z = quat.z();
  current_pose.pose.pose.orientation.w = quat.w();

  current_pose.pose.covariance[0] = 0.001;
  current_pose.pose.covariance[7] = 0.001;
  current_pose.pose.covariance[35] = 0.001;

  // Message for corrected got_pose
  geometry_msgs::PoseWithCovarianceStamped pose_corrected;

  pose_corrected.header.seq = current_pose.header.seq;
  pose_corrected.header.stamp = current_pose.header.stamp;
  pose_corrected.header.frame_id = "odom";

  pose_corrected.pose.pose.position.x = got_x_ + error_x;
  pose_corrected.pose.pose.position.y = got_y_ + error_y;
  pose_corrected.pose.pose.position.z = got_z_;

  pose_corrected.pose.pose.orientation.x = quat.x();
  pose_corrected.pose.pose.orientation.y = quat.y();
  pose_corrected.pose.pose.orientation.z = quat.z();
  pose_corrected.pose.pose.orientation.w = quat.w();

  pose_corrected.pose.covariance[0] = 0.001;
  pose_corrected.pose.covariance[7] = 0.001;
  pose_corrected.pose.covariance[35] = 0.001;

  got_pub_.publish(current_pose);
  got_corrected_pub_.publish(pose_corrected);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "got_position_node");
  TeleopTurtle teleop_turtle;

  ros::spin();
}