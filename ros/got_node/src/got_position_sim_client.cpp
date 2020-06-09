/* ROS node that aims to simulate GoT position
*
* - Subscribe to /got_teensy (geometry_msgs/Pose) published by teensy
*   
* - Service call current position to "get_pos_error" service to get error
* 
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

#include "got_node/GetPosError.h"

#include <fstream>
#include <iostream>

#define NUM_BEACONS 5
//#define WRITE_TO_FILE


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void gotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  ros::NodeHandle nh_;

  ros::Publisher got_raw_pub_;
  ros::Publisher got_corr_pub_;

  ros::Subscriber odom_sub_;

  ros::ServiceClient client_;

  const double beacon_locations_[NUM_BEACONS][3] = { {-2,4,5}, {-2,-4,5}, {2,0,5}, {6,4,5}, {6,-4,5}};
  
  unsigned frame_sequence_;

  double error_x_ = 0, error_y_ = 0;

};


TeleopTurtle::TeleopTurtle()
{
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("ground_truth", 10, &TeleopTurtle::groundTruthCallback, this);

  got_raw_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_raw", 1);
  got_corr_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_corr", 1);

  client_ = nh_.serviceClient<got_node::GetPosError>("/master/get_pos_error");
}

void TeleopTurtle::groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double ground_x = msg->pose.pose.position.x;
  double ground_y = msg->pose.pose.position.y;
  double ground_z = msg->pose.pose.position.y;

  double got_x = 0, got_y = 0, got_z = 0;

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

      double dp=pow(got_x- beacon_locations_[index][0], 2);
      dp+=pow(got_y- beacon_locations_[index][1], 2);
      dp+=pow(got_z- beacon_locations_[index][2], 2);
      dp=sqrt(dp);

      // Normalizing factor
      double lambda = 1 - dp / distance;
      //ROS_INFO("Lambda: %f", lambda);

      // Estimated Cartesian coordinates
      got_x=got_x/(1-lambda)-beacon_locations_[index][0]*lambda/(1-lambda);
      got_y=got_y/(1-lambda)-beacon_locations_[index][1]*lambda/(1-lambda);
      got_z=got_z/(1-lambda)-beacon_locations_[index][2]*lambda/(1-lambda);
      //ROS_INFO("Position: %d, %f, %f, %f;", index, x_est, y_est, z_est);
  }

  got_node::GetPosError srv;

  // float key_x = ((int)(ground_x * 1000)) / 1000.0;   // Store error for every 0.1 mm
  // float key_y = ((int)(ground_y * 1000)) / 1000.0;
  float key_x = ((int)(ground_x * 10)) / 10.0;   // Store error for every 1 mm (0.001 m)
  float key_y = ((int)(ground_y * 10)) / 10.0;

  // key_x = 0.5 * ((int) (key_x / 0.5));
  // key_y = 0.5 * ((int) (key_y / 0.5));

  srv.request.x = key_x;
  srv.request.y = key_y;

  if (client_.call(srv))
  {
    error_x_ = srv.response.err_x;
    error_y_ = srv.response.err_y;
    ROS_INFO("Server response: %f, %f", srv.response.err_x, srv.response.err_y);
  }
  else
  {
    //Previous error will stay
    ROS_ERROR("Failed to call service get_pos_error");
  }

  tf2::Quaternion quat;
  quat.setRPY(0,0,0);

  geometry_msgs::PoseWithCovarianceStamped got_raw_pose;

  //got_raw_pose.header.seq = ++ frame_sequence_;
  got_raw_pose.header.stamp = ros::Time::now();
  got_raw_pose.header.frame_id = "odom";
  got_raw_pose.pose.pose.position.x = got_x;
  got_raw_pose.pose.pose.position.y = got_y;
  got_raw_pose.pose.pose.position.z = 0.0;
  got_raw_pose.pose.pose.orientation.x = quat.x();
  got_raw_pose.pose.pose.orientation.y = quat.y();
  got_raw_pose.pose.pose.orientation.z = quat.z();
  got_raw_pose.pose.pose.orientation.w = quat.w();
  got_raw_pose.pose.covariance[0] = 0.001;
  got_raw_pose.pose.covariance[7] = 0.001;
  got_raw_pose.pose.covariance[35] = 0.001;

  geometry_msgs::PoseWithCovarianceStamped got_cor_pose;

  //got_cor_pose.header.seq = ++ frame_sequence_;
  got_cor_pose.header.stamp = ros::Time::now();
  got_cor_pose.header.frame_id = "odom";
  got_cor_pose.pose.pose.position.x = got_x + error_x_;
  got_cor_pose.pose.pose.position.y = got_y + error_y_;
  got_cor_pose.pose.pose.position.z = 0.0;
  got_cor_pose.pose.pose.orientation.x = quat.x();
  got_cor_pose.pose.pose.orientation.y = quat.y();
  got_cor_pose.pose.pose.orientation.z = quat.z();
  got_cor_pose.pose.pose.orientation.w = quat.w();
  got_cor_pose.pose.covariance[0] = 0.001;
  got_cor_pose.pose.covariance[7] = 0.001;
  got_cor_pose.pose.covariance[35] = 0.001;

  got_raw_pub_.publish(got_raw_pose);
  got_corr_pub_.publish(got_cor_pose);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "got_position_node_client");
  TeleopTurtle teleop_turtle;

  ros::spin();
}