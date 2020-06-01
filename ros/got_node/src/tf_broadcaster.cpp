#include <ros/ros.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_broadcaster.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <fstream>
#include <iostream>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  void gotTeensyCallback(const geometry_msgs::Point::ConstPtr& msg);
  void gotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  
  void odomRawCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomLocalCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomGlobalGOTRawCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomGlobalGOTCorrCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void odomLocalSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomGlobalGOTCRawSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomGlobalGOTCorrSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg);

  
  void slamRawCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void slamLocalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void slamGlobalGOTRawCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void slamGlobalGOTCorrCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void goalPositionCallback(const geometry_msgs::Point::ConstPtr& msg);
  
  ros::NodeHandle nh_;

  ros::Publisher scan_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher got_raw_pub_;
  ros::Publisher slam_pub_;

  ros::Subscriber scan_sub_;
  ros::Subscriber imu_sub_;

  ros::Subscriber got_teensy_sub_;
  ros::Subscriber got_corrected_sub_;

  ros::Subscriber odom_raw_sub_;  
  ros::Subscriber odom_local_sub_;
  ros::Subscriber odom_global_got_raw_sub_;
  ros::Subscriber odom_global_got_corr_sub_;

  ros::Subscriber odom_local_slam_sub_;
  ros::Subscriber odom_global_got_raw_slam_sub_;
  ros::Subscriber odom_global_got_corr_slam_sub_;

  ros::Subscriber slam_raw_sub_;
  ros::Subscriber slam_local_sub_;
  ros::Subscriber slam_global_got_raw_sub_;
  ros::Subscriber slam_global_got_corr_sub_;

  ros::Subscriber goal_position_sub_;

  geometry_msgs::Point got_teensy_pose_;
  geometry_msgs::Point got_corrected_pose_;

  // odom raw Available in default Callback
  geometry_msgs::Point odom_local_pose_;
  geometry_msgs::Point odom_global_got_raw_pose_;
  geometry_msgs::Point odom_global_got_corr_pose_;

  geometry_msgs::Point odom_local_slam_pose_;
  geometry_msgs::Point odom_global_got_raw_slam_pose_;
  geometry_msgs::Point odom_global_got_corr_slam_pose_;

  geometry_msgs::Point slam_raw_pose_;
  geometry_msgs::Point slam_local_pose_;
  geometry_msgs::Point slam_global_got_raw_pose_;
  geometry_msgs::Point slam_global_got_corr_pose_;

  geometry_msgs::Point goal_position_pose_;

  bool param_pub_odom_tf_ = true;
  bool param_pub_odom_topic_= true;
  bool param_write_to_file_ = false;
  std::string namespace_ = "";

  bool offset_obtained_ = false;
  double offset_x_ = 0;
  double offset_y_ = 0;

};


TeleopTurtle::TeleopTurtle()
{
  ros::param::param<std::string>("~namespace", namespace_, "");
  ros::param::param<bool>("~publishOdomTF", param_pub_odom_tf_, true);
  ros::param::param<bool>("~publishOdomTopic", param_pub_odom_topic_, true);
  ros::param::param<bool>("~writeToFile", param_write_to_file_, false);

  odom_raw_sub_ = nh_.subscribe<nav_msgs::Odometry>("/raw_odom", 10, &TeleopTurtle::odomRawCallback, this);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &TeleopTurtle::laserCallback, this);
  imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 10, &TeleopTurtle::imuCallback, this);
  got_teensy_sub_ = nh_.subscribe<geometry_msgs::Point>("/got_teensy", 10, &TeleopTurtle::gotTeensyCallback, this);

  slam_raw_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 10, &TeleopTurtle::slamRawCallback, this);

  scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("raw_odom", 1);
  slam_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_pose_cov", 1);
  
  got_raw_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_raw", 1);

  if (param_write_to_file_)
  {
    got_corrected_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("got_pose_corrected_v2", 10, &TeleopTurtle::gotCallback, this);
    
    odom_local_sub_ = nh_.subscribe<nav_msgs::Odometry>("/fused/odom_local", 10, &TeleopTurtle::odomLocalCallback, this);
    odom_global_got_raw_sub_ = nh_.subscribe<nav_msgs::Odometry>("/fused_got_raw/odom", 10, &TeleopTurtle::odomGlobalGOTRawCallback, this);
    odom_global_got_corr_sub_ = nh_.subscribe<nav_msgs::Odometry>("/fused_got_corr/odom", 10, &TeleopTurtle::odomGlobalGOTCorrCallback, this);

    odom_local_slam_sub_ = nh_.subscribe<nav_msgs::Odometry>("/fused/odom", 10, &TeleopTurtle::odomLocalSLAMCallback, this);
    odom_global_got_raw_slam_sub_ = nh_.subscribe<nav_msgs::Odometry>("/fused_got_raw_slam/odom", 10, &TeleopTurtle::odomGlobalGOTCRawSLAMCallback, this);
    odom_global_got_corr_slam_sub_ = nh_.subscribe<nav_msgs::Odometry>("/fused_got_corr_slam/odom", 10, &TeleopTurtle::odomGlobalGOTCorrSLAMCallback, this);

    //slam_raw_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/raw/slam_out_pose", 10, &TeleopTurtle::slamRawCallback, this);

    //slam_local_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/fused/slam_out_pose", 10, &TeleopTurtle::slamLocalCallback, this);
    //slam_global_got_raw_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/fused_got_raw/slam_out_pose", 10, &TeleopTurtle::slamGlobalGOTRawCallback, this);
    //slam_global_got_corr_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/fused_got_corr/slam_out_pose", 10, &TeleopTurtle::slamGlobalGOTCorrCallback, this);

    goal_position_sub_ = nh_.subscribe<geometry_msgs::Point>("/goal_position_mb", 10, &TeleopTurtle::goalPositionCallback, this);
  }
}

void TeleopTurtle::gotTeensyCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  // ROS_INFO("1.got_teensy");
  // TODO: For rosbag test the robot is moving towards negative x-axis according to GoT
  got_teensy_pose_.x = -(msg->x);
  got_teensy_pose_.y = -(msg->y);
  got_teensy_pose_.z = -(msg->z);

  geometry_msgs::PoseWithCovarianceStamped got_pose;
  tf2::Quaternion quat;
  quat.setRPY(0,0,0);

  //got_pose.header.seq = ++ frame_sequence_;
  got_pose.header.stamp = ros::Time::now();
  got_pose.header.frame_id = namespace_+"/odom";
  got_pose.pose.pose.position.x = -(msg->x);
  got_pose.pose.pose.position.y = -(msg->y);
  got_pose.pose.pose.position.z = 0.0;
  got_pose.pose.pose.orientation.x = quat.x();
  got_pose.pose.pose.orientation.y = quat.y();
  got_pose.pose.pose.orientation.z = quat.z();
  got_pose.pose.pose.orientation.w = quat.w();

  got_pose.pose.covariance[0] = 0.001;
  got_pose.pose.covariance[7] = 0.001;
  got_pose.pose.covariance[35] = 0.001;

  got_raw_pub_.publish(got_pose);
}

void TeleopTurtle::goalPositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  // ROS_INFO("1.got_teensy");
  // TODO: For rosbag test the robot is moving towards negative x-axis according to GoT
  goal_position_pose_.x = msg->x;
  goal_position_pose_.y = msg->y;
  goal_position_pose_.z = msg->z;
}

void TeleopTurtle::gotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // ROS_INFO("2.got_corrected");
  got_corrected_pose_.x = msg->pose.pose.position.x;
  got_corrected_pose_.y = msg->pose.pose.position.y;
  got_corrected_pose_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::odomLocalCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // ROS_INFO("3.odom_local");
  odom_local_pose_.x = msg->pose.pose.position.x;
  odom_local_pose_.y = msg->pose.pose.position.y;
  odom_local_pose_.z = 0.0;
}

void TeleopTurtle::odomGlobalGOTRawCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // ROS_INFO("4.odom_global_got_raw");
  odom_global_got_raw_pose_.x = msg->pose.pose.position.x;
  odom_global_got_raw_pose_.y = msg->pose.pose.position.y;
  odom_global_got_raw_pose_.z = 0.0;
}

void TeleopTurtle::odomGlobalGOTCorrCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // ROS_INFO("5.odom_global_got_corr");
  odom_global_got_corr_pose_.x = msg->pose.pose.position.x;
  odom_global_got_corr_pose_.y = msg->pose.pose.position.y;
  odom_global_got_corr_pose_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::odomLocalSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_local_slam_pose_.x = msg->pose.pose.position.x;
  odom_local_slam_pose_.y = msg->pose.pose.position.y;
  odom_local_slam_pose_.z = 0.0;
}

void TeleopTurtle::odomGlobalGOTCRawSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_global_got_raw_slam_pose_.x = msg->pose.pose.position.x;
  odom_global_got_raw_slam_pose_.y = msg->pose.pose.position.y;
  odom_global_got_raw_slam_pose_.z = 0.0;
}

void TeleopTurtle::odomGlobalGOTCorrSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_global_got_corr_slam_pose_.x = msg->pose.pose.position.x;
  odom_global_got_corr_slam_pose_.y = msg->pose.pose.position.y;
  odom_global_got_corr_slam_pose_.z = 0.0;
}


void TeleopTurtle::slamRawCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // ROS_INFO("6.slam_raw");
  slam_raw_pose_.x = msg->pose.position.x;
  slam_raw_pose_.y = msg->pose.position.y;
  slam_raw_pose_.z = msg->pose.position.z;

  geometry_msgs::PoseWithCovarianceStamped slam_pose;
  //got_pose.header.seq = ++ frame_sequence_;
  slam_pose.header.stamp = ros::Time::now();
  slam_pose.header.frame_id = namespace_+"/map";
  slam_pose.pose.pose.position.x = msg->pose.position.x;
  slam_pose.pose.pose.position.y = msg->pose.position.y;
  slam_pose.pose.pose.position.z = 0.0;
  slam_pose.pose.pose.orientation.x = msg->pose.orientation.x;
  slam_pose.pose.pose.orientation.y = msg->pose.orientation.y;
  slam_pose.pose.pose.orientation.z = msg->pose.orientation.z;
  slam_pose.pose.pose.orientation.w = msg->pose.orientation.w;
  slam_pose.pose.covariance[0] = 0.001;
  slam_pose.pose.covariance[7] = 0.001;
  slam_pose.pose.covariance[35] = 0.001;

  slam_pub_.publish(slam_pose);

}

void TeleopTurtle::slamLocalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // ROS_INFO("7.slam_local");
  slam_local_pose_.x = msg->pose.position.x;
  slam_local_pose_.y = msg->pose.position.y;
  slam_local_pose_.z = msg->pose.position.z;
}

void TeleopTurtle::slamGlobalGOTRawCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // ROS_INFO("8.slam_global_got_raw");
  slam_global_got_raw_pose_.x = msg->pose.position.x;
  slam_global_got_raw_pose_.y = msg->pose.position.y;
  slam_global_got_raw_pose_.z = msg->pose.position.z;
}

void TeleopTurtle::slamGlobalGOTCorrCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // ROS_INFO("9.slam_global_got_corr");
  slam_global_got_corr_pose_.x = msg->pose.position.x;
  slam_global_got_corr_pose_.y = msg->pose.position.y;
  slam_global_got_corr_pose_.z = msg->pose.position.z;
}


void TeleopTurtle::odomRawCallback(const nav_msgs::Odometry::ConstPtr& msg){

  //ROS_INFO("Got namespace: %s", namespace_.c_str());

  // Publish odom -> base TF
  if (param_pub_odom_tf_)
  {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
    tf::Quaternion q;
    //q.setRPY(0, 0, msg->theta);
    q.setValue(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    transform.setRotation(q);
    //br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, namespace_+"/odom", namespace_+"/base_footprint"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), namespace_+"/odom", namespace_+"/base_footprint"));
  }

  // Publish Odometry topic
  if (param_pub_odom_topic_)
  {
    nav_msgs::Odometry odom;
    // odom.header.stamp = msg->header.stamp;
    // odom.header.seq = msg->header.seq;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = namespace_+"/odom";
    odom.child_frame_id = namespace_+"/base_footprint";
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

    odom_pub_.publish(odom);
  }

  if(param_write_to_file_)
  {
    if (!offset_obtained_)
    {
      //if (odom_pose.pose.pose.position.x != 0) // Wait to get actual data as it takes a while to get a reading sometimes
      if (got_teensy_pose_.x != 0) // Wait to get actual data as it takes a while to get a reading sometimes
      {
        offset_x_ = odom_local_pose_.x - got_teensy_pose_.x;
        offset_y_ = odom_local_pose_.y - got_teensy_pose_.y;
        offset_obtained_ = true;
      }
    }else
    {
      std::ofstream myfile;
      myfile.open ("/home/isircu/err_log.csv", std::ios::out | std::ios::app);

      // myfile << odom_local_pose_.x - msg->pose.pose.position.x << ','  // Raw odom error
      //        << odom_local_pose_.y - msg->pose.pose.position.y << ','
      //        << odom_local_pose_.x - odom_global_got_raw_pose_.x << ','  // Global_got_raw odom error
      //        << odom_local_pose_.y - odom_global_got_raw_pose_.y << ','
      //        << odom_local_pose_.x - odom_global_got_corr_pose_.x << ','  // Global_got_corrected odom error
      //        << odom_local_pose_.y - odom_global_got_corr_pose_.y << ','
      //        << odom_local_pose_.x - slam_raw_pose_.x << ','              // Slam raw error
      //        << odom_local_pose_.y - slam_raw_pose_.y << ','
      //        << odom_local_pose_.x - slam_local_pose_.x << ','            // Slam local error
      //        << odom_local_pose_.y - slam_local_pose_.y << ','
      //        << odom_local_pose_.x - slam_global_got_raw_pose_.x << ','  // Slam global got_raw error
      //        << odom_local_pose_.y - slam_global_got_raw_pose_.y << ','
      //        << odom_local_pose_.x - slam_global_got_corr_pose_.x << ','  // Slam global got_raw error
      //        << odom_local_pose_.y - slam_global_got_corr_pose_.y << ','
      //        << odom_local_pose_.x - (got_teensy_pose_.x + offset_x_) << ','  // Got raw error
      //        << odom_local_pose_.y - (got_teensy_pose_.y + offset_y_) << ','
      //        << odom_local_pose_.x - got_corrected_pose_.x << ','           //Got corrected error
      //        << odom_local_pose_.y - got_corrected_pose_.y << '\n';


      // myfile << odom_local_pose_.x << ','  // Ground truth
      //        << odom_local_pose_.y << ','
      //        << msg->pose.pose.position.x << ','    //Raw odom
      //        << msg->pose.pose.position.y << ','
      //        << odom_global_got_raw_pose_.x << ','  // Global_got_raw odom error
      //        << odom_global_got_raw_pose_.y << ','
      //        << odom_global_got_corr_pose_.x << ','  // Global_got_corrected odom error
      //        << odom_global_got_corr_pose_.y << ','
      //        << slam_raw_pose_.x << ','              // Slam raw error
      //        << slam_raw_pose_.y << ','
      //        << slam_local_pose_.x << ','            // Slam local error
      //        << slam_local_pose_.y << ','
      //        << slam_global_got_raw_pose_.x << ','  // Slam global got_raw error
      //        << slam_global_got_raw_pose_.y << ','
      //        << slam_global_got_corr_pose_.x << ','  // Slam global got_raw error
      //        << slam_global_got_corr_pose_.y << ','
      //        << (got_teensy_pose_.x + offset_x_) << ','  // Got raw error
      //        << (got_teensy_pose_.y + offset_y_) << ','
      //        << got_corrected_pose_.x << ','           //Got corrected error
      //        << got_corrected_pose_.y << '\n';

      myfile << goal_position_pose_.x << ','        // Ground truth (move_base goal position)
             << goal_position_pose_.y << ','
             << msg->pose.pose.position.x << ','    //Raw odom
             << msg->pose.pose.position.y << ','
             << odom_local_pose_.x << ','           // Odom+imu
             << odom_local_pose_.y << ','
             << odom_global_got_raw_pose_.x << ','  // Odom + imu + got_raw
             << odom_global_got_raw_pose_.y << ','
             << odom_global_got_corr_pose_.x << ','  // Global_got_corrected odom error
             << odom_global_got_corr_pose_.y << ','
             << slam_raw_pose_.x << ','              // Slam raw error
             << slam_raw_pose_.y << ','
             << odom_local_slam_pose_.x << ','      // SLAM +odom + imu -> EKF->
             << odom_local_slam_pose_.y << ','
             << odom_global_got_raw_slam_pose_.x << ','      // SLAM +odom + imu +got_raw -> EKF->
             << odom_global_got_raw_slam_pose_.y << ','
             << odom_global_got_corr_slam_pose_.x << ','      // SLAM +odom + imu +got_raw -> EKF->
             << odom_global_got_corr_slam_pose_.y << ','
             << (got_teensy_pose_.x + offset_x_) << ','  // Got raw error
             << (got_teensy_pose_.y + offset_y_) << ','
             << got_corrected_pose_.x << ','           //Got corrected error
             << got_corrected_pose_.y << '\n';

      // myfile << msg->pose.pose.position.x << ','  // Raw odom
      //        << msg->pose.pose.position.y << ','
      //        << (got_teensy_pose_.x + offset_x_) << ','  // Got raw with offset
      //        << (got_teensy_pose_.y + offset_y_) << '\n';

      myfile.close();
    }    
  }
}

void TeleopTurtle::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  sensor_msgs::LaserScan laser_msg;

  laser_msg.header.stamp = ros::Time::now();
  // laser_msg.header.stamp = msg->header.stamp;
  laser_msg.header.frame_id = namespace_+"/laser";
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

void TeleopTurtle::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  sensor_msgs::Imu imu_msg;

  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = namespace_+"/imu_link";
  imu_msg.orientation.x = msg->orientation.x;
  imu_msg.orientation.y = msg->orientation.y;
  imu_msg.orientation.z = msg->orientation.z;
  imu_msg.orientation.w = msg->orientation.w;
  imu_msg.orientation_covariance = msg->orientation_covariance;
  imu_msg.angular_velocity.x = msg->angular_velocity.x;
  imu_msg.angular_velocity.y = msg->angular_velocity.y;
  imu_msg.angular_velocity.z = msg->angular_velocity.z;
  imu_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu_msg.linear_acceleration.x = msg->linear_acceleration.x;
  imu_msg.linear_acceleration.y = msg->linear_acceleration.y;
  imu_msg.linear_acceleration.z = msg->linear_acceleration.z;
  imu_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;

  imu_pub_.publish(imu_msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster");
  TeleopTurtle teleop_turtle;

  ros::spin();
}