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

  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;

};


TeleopTurtle::TeleopTurtle()
{
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/raw_odom", 10, &TeleopTurtle::odomCallback, this);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &TeleopTurtle::laserCallback, this);

  scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("raw_scan", 1);
}


void TeleopTurtle::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  tf::Quaternion q;
  //q.setRPY(0, 0, msg->theta);
  q.setValue(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  transform.setRotation(q);
  //br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom_raw", "base_footprint_raw"));
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_raw", "base_footprint_raw"));

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  //static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.stamp = msg->header.stamp;
  static_transformStamped.header.seq = msg->header.seq;
  static_transformStamped.header.frame_id = "base_footprint_raw";
  static_transformStamped.child_frame_id = "laser_raw";
  static_transformStamped.transform.translation.x = 0.110;
  static_transformStamped.transform.translation.y = 0.0;
  static_transformStamped.transform.translation.z = 0.2;
  tf2::Quaternion quat;
  quat.setRPY(0,0,0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();


  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom_raw", "base_footprint_raw"));
  static_broadcaster.sendTransform(static_transformStamped);
}

void TeleopTurtle::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  sensor_msgs::LaserScan laser_msg;

  //laser_msg.header.stamp = ros::Time::now();
  laser_msg.header.stamp = msg->header.stamp;
  laser_msg.header.seq = msg->header.seq;
  laser_msg.header.frame_id = "laser_raw";
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
  ros::init(argc, argv, "tf_broadcaster_raw");
  TeleopTurtle teleop_turtle;

  ros::spin();
}