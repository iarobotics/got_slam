#include <ros/ros.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/LaserScan.h>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  ros::NodeHandle nh_;

  ros::Publisher scan_pub_;
  ros::Publisher odom_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  

};


TeleopTurtle::TeleopTurtle()
{
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/raw_odom", 10, &TeleopTurtle::odomCallback, this);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &TeleopTurtle::laserCallback, this);

  scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("filtered_scan", 1);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("filtered_raw_odom", 1);
}


void TeleopTurtle::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

  nav_msgs::Odometry odom;

  odom.header.stamp = msg->header.stamp;
  odom.header.seq = msg->header.seq;
  odom.header.frame_id = "odom_filtered";
  odom.child_frame_id = "base_footprint_filtered";
  odom.pose.pose.position.x = msg->pose.pose.position.x;
  odom.pose.pose.position.y = msg->pose.pose.position.y;
  odom.pose.pose.position.z = msg->pose.pose.position.z;
  odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  odom.pose.covariance = msg->pose.covariance;
  odom.twist.twist.linear.x = msg->twist.twist.linear.x;
  odom.twist.twist.linear.y = msg->twist.twist.linear.y;
  odom.twist.twist.linear.z = msg->twist.twist.linear.z;
  odom.twist.twist.angular.x = msg->twist.twist.angular.x;
  odom.twist.twist.angular.y = msg->twist.twist.angular.y;
  odom.twist.twist.angular.z = msg->twist.twist.angular.z;
  odom.twist.covariance = msg->twist.covariance;

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  //static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.stamp = msg->header.stamp;
  static_transformStamped.header.seq = msg->header.seq;
  static_transformStamped.header.frame_id = "base_footprint_filtered";
  static_transformStamped.child_frame_id = "laser_filtered";
  static_transformStamped.transform.translation.x = 0.110;
  static_transformStamped.transform.translation.y = 0.0;
  static_transformStamped.transform.translation.z = 0.2;
  tf2::Quaternion quat;
  quat.setRPY(0,0,0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();

  odom_pub_.publish(odom);
  static_broadcaster.sendTransform(static_transformStamped);
}

void TeleopTurtle::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  sensor_msgs::LaserScan laser_msg;

  //laser_msg.header.stamp = ros::Time::now();
  laser_msg.header.stamp = msg->header.stamp;
  laser_msg.header.frame_id = "laser_filtered";
  laser_msg.angle_min = msg->angle_min;
  laser_msg.angle_max = msg->angle_max;
  laser_msg.angle_increment = msg->angle_increment;
  laser_msg.time_increment = msg->time_increment;
  laser_msg.scan_time = msg->scan_time;
  laser_msg.range_min = msg->range_min;
  laser_msg.range_max = msg->range_max;
  laser_msg.ranges = msg->ranges;
  laser_msg.intensities = msg->intensities;

  scan_pub_.publish(laser_msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster_filtered");
  TeleopTurtle teleop_turtle;

  ros::spin();
}