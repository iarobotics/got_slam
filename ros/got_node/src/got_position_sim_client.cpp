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

  ros::Publisher got_pub_;

  ros::Subscriber odom_sub_;

  ros::ServiceClient client_;

  const double beacon_locations_[NUM_BEACONS][3] = { {-2,4,5}, {-2,-4,5}, {2,0,5}, {6,4,5}, {6,-4,5}};
  
  unsigned frame_sequence_;

};


TeleopTurtle::TeleopTurtle()
{
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("ground_truth", 10, &TeleopTurtle::groundTruthCallback, this);

  got_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_pose_corrected_client", 1);

  client_ = nh_.serviceClient<got_node::GetPosError>("/robot1/get_pos_error");
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
      
      // myfile << ground_x -slam_pos_.x << ','
      //        << ground_y -slam_pos_.y << ','
      //        << slam_pos_.x - odom_filtered_local_.x << ','
      //        << slam_pos_.y - odom_filtered_local_.y << ','
      //        << slam_pos_.x - odom_filtered_.x << ','
      //        << slam_pos_.y - odom_filtered_.y << ','
      //        << slam_pos_.x - got_x_ << ','
      //        << slam_pos_.y - got_y_ << '\n';

      myfile.close();
    #endif

  got_node::GetPosError srv;

  float got_key_x = ((int)(msg->pose.pose.position.x * 10)) / 10.0;
  float got_key_y = ((int)(msg->pose.pose.position.x * 10)) / 10.0;

  srv.request.x = got_key_x;
  srv.request.y = got_key_y;

  double error_x = 0, error_y = 0;

  if (client_.call(srv))
  {
    error_x = srv.response.err_x;
    error_y = srv.response.err_y;
    ROS_INFO("Server response: %f, %f", srv.response.err_x, srv.response.err_y);
  }
  else
  {
    ROS_ERROR("Failed to call service get_pos_error");
  }


  geometry_msgs::PoseWithCovarianceStamped current_pose;
  tf2::Quaternion quat;
  quat.setRPY(0,0,0);

  current_pose.header.seq = ++ frame_sequence_;
  current_pose.header.stamp = ros::Time::now();
  current_pose.header.frame_id = "odom";

  // current_pose.pose.pose.position.x = msg->pose.pose.position.x;
  // current_pose.pose.pose.position.y = msg->pose.pose.position.y;
  // current_pose.pose.pose.position.z = msg->pose.pose.position.z;

  current_pose.pose.pose.position.x = got_x_ + error_x;
  current_pose.pose.pose.position.y = got_y_ + error_y;
  current_pose.pose.pose.position.z = 0.0;

  current_pose.pose.pose.orientation.x = quat.x();
  current_pose.pose.pose.orientation.y = quat.y();
  current_pose.pose.pose.orientation.z = quat.z();
  current_pose.pose.pose.orientation.w = quat.w();

  current_pose.pose.covariance[0] = 0.001;
  current_pose.pose.covariance[7] = 0.001;
  current_pose.pose.covariance[35] = 0.001;

  got_pub_.publish(current_pose);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "got_position_node_client");
  TeleopTurtle teleop_turtle;

  ros::spin();
}