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

bool sendErr(got_node::GetPosError::Request  &req,
         got_node::GetPosError::Response &res)
{
  res.err_x = 12.3;
  res.err_y = 1.0;
  ROS_INFO("request: x=%f, y=%f", req.x, req.y);
  ROS_INFO("sending back response: ex:[%f] ey:[%f]", res.err_x, res.err_y);
  return true;
}


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void teensyCallback(const geometry_msgs::Point::ConstPtr& msg);

  ros::NodeHandle nh_;
  
  ros::Subscriber teensy_sub_;

  ros::Publisher got_pub_;

  ros::ServiceClient client_;
  
  unsigned frame_sequence_;

  bool offset_obtained_ = false;
  double offset_x_ = 0;
  double offset_y_ = 0;
};


TeleopTurtle::TeleopTurtle()
{
  teensy_sub_ = nh_.subscribe<geometry_msgs::Point>("got_teensy", 10, &TeleopTurtle::teensyCallback, this);

  got_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_pose_corrected", 1);

  client_ = nh_.serviceClient<got_node::GetPosError>("/robot1/get_pos_error");
}


void TeleopTurtle::teensyCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  double got_x = msg->x;
  double got_y = msg->y;

  if (!offset_obtained_)
  {
    //if (odom_pose.pose.pose.position.x != 0) // Wait to get actual data as it takes a while to get a reading sometimes
    if (got_x != 0) // Wait to get actual data as it takes a while to get a reading sometimes
    {
      offset_x_ = -got_x;
      offset_y_ = -got_y;
      offset_obtained_ = true;
    }
  } else
  {
    //Remove offset
    got_x += offset_x_;
    got_y += offset_y_;

    double key_x = ((int)(got_x * 10)) / 10;   // Store error for every 100 mm
    double key_y = ((int)(got_y * 10)) / 10;

    got_node::GetPosError srv;
    srv.request.x = key_x;
    srv.request.y = key_y;

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

    // Construct and publish a new GoT message with error correction
    geometry_msgs::PoseWithCovarianceStamped got_pose;
    tf2::Quaternion quat;
    quat.setRPY(0,0,0);

    //got_pose.header.seq = ++ frame_sequence_;
    got_pose.header.stamp = ros::Time::now();
    got_pose.header.frame_id = "odom";
    got_pose.pose.pose.position.x = got_x + error_x;
    got_pose.pose.pose.position.y = got_y + error_y;
    got_pose.pose.pose.position.z = 0;
    got_pose.pose.pose.orientation.x = quat.x();
    got_pose.pose.pose.orientation.y = quat.y();
    got_pose.pose.pose.orientation.z = quat.z();
    got_pose.pose.pose.orientation.w = quat.w();

    got_pose.pose.covariance[0] = 0.001;
    got_pose.pose.covariance[7] = 0.001;
    got_pose.pose.covariance[35] = 0.001;

    got_pub_.publish(got_pose);

  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "got_position_node_client");
  TeleopTurtle teleop_turtle;

  ros::spin();
}