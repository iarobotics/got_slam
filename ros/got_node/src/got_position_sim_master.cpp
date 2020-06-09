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

#include <tf/transform_broadcaster.h>

#include "got_node/GetPosError.h"

#include <fstream>
#include <iostream>

#define NUM_BEACONS 5
//#define WRITE_TO_FILE

struct Position { 
	float x, y; 

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
  void rawOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomLocalCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomGlobalGOTRawCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomGlobalGOTCorrCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void slamCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void odomGlobalSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomGlobalGOTRawSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void odomGlobalGOTCorrSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void client_groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void client_odomLocalCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void client_odomGlobalGOTRawCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void client_odomGlobalGOTCorrCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void client_gotRaw(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void client_gotCorr(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  bool sendErr(got_node::GetPosError::Request  &req,
         got_node::GetPosError::Response &res);
  
  ros::NodeHandle nh_;

  ros::Publisher got_pub_;
  ros::Publisher got_corrected_pub_;
  ros::Publisher slam_pub_;
  
  
  ros::Subscriber odom_raw_sub_;
  ros::Subscriber ground_truth_sub_;
  ros::Subscriber odom_local_sub_;
  ros::Subscriber odom_global_got_raw_sub_;
  ros::Subscriber odom_global_got_corr_sub_;
  ros::Subscriber slam_sub_;
  ros::Subscriber odom_global_slam_sub_;
  ros::Subscriber odom_global_got_raw_slam_sub_;
  ros::Subscriber odom_global_got_corr_slam_sub_;

  ros::Subscriber client_ground_truth_sub_;
  ros::Subscriber client_odom_local_sub_;
  ros::Subscriber client_odom_global_got_raw_sub_;
  ros::Subscriber client_odom_global_got_corr_sub_;
  ros::Subscriber client_got_raw_sub_;
  ros::Subscriber client_got_corr_sub_;
  
  unsigned frame_sequence_;

  //double got_x=0, got_y=0, got_z=0;

  const double beacon_locations_[NUM_BEACONS][3] = { {-2,4,5}, {-2,-4,5}, {2,0,5}, {6,4,5}, {6,-4,5}};

  //Parameters
  double err_s_ = 400;
  double err_tolerance_ = 0.2; // 10 cm

  std::string namespace_ = "";
  bool param_write_to_file_ = false;

  geometry_msgs::Point odom_raw_pos_;
  //odom_ground truth is part of groundTruthCallback()
  geometry_msgs::Point odom_local_pos_;
  geometry_msgs::Point odom_global_got_raw_pos_;
  geometry_msgs::Point odom_global_got_corr_pos_;
  geometry_msgs::Point slam_pos_;
  geometry_msgs::Point odom_global_slam_pos_;
  geometry_msgs::Point odom_global_got_raw_slam_pos_;
  geometry_msgs::Point odom_global_got_corr_slam_pos_;

  geometry_msgs::Point client_ground_pos_;
  geometry_msgs::Point client_odom_local_pos_;
  geometry_msgs::Point client_odom_global_got_raw_pos_;
  geometry_msgs::Point client_odom_global_got_corr_pos_;
  geometry_msgs::Point client_got_raw_pos_;
  geometry_msgs::Point client_got_corr_pos_;

  std::unordered_map<Position, Position, MyHashFunction> error_map_;

  ros::ServiceServer service_;
};


TeleopTurtle::TeleopTurtle()
{
  ros::param::param<std::string>("~namespace", namespace_, "");
  if (namespace_ != "")
  {
    namespace_ += "/";
  }
  ros::param::param<std::string>("~namespace", namespace_, "");
  ros::param::param<double>("~got_err_tolerance", err_tolerance_, 0.2);
  ros::param::param<double>("~got_err_s", err_s_, 400);
  ros::param::param<bool>("~writeToFile", param_write_to_file_, false);

  //got_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_pose", 1);
  got_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_raw", 1);
  got_corrected_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_corr", 1);
  slam_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_pose_cov", 1);

  odom_raw_sub_ = nh_.subscribe<nav_msgs::Odometry>("raw_odom", 10, &TeleopTurtle::rawOdomCallback, this);
  ground_truth_sub_ = nh_.subscribe<nav_msgs::Odometry>("ground_truth", 10, &TeleopTurtle::groundTruthCallback, this);
  odom_local_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom_local", 10, &TeleopTurtle::odomLocalCallback, this);
  odom_global_got_raw_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom_global_got_raw", 10, &TeleopTurtle::odomGlobalGOTRawCallback, this);
  odom_global_got_corr_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom_global_got_corr", 10, &TeleopTurtle::odomGlobalGOTCorrCallback, this);

  slam_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 10, &TeleopTurtle::slamCallback, this);
  odom_global_slam_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom_global_slam", 10, &TeleopTurtle::odomGlobalSLAMCallback, this);
  odom_global_got_raw_slam_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom_global_got_raw_slam", 10, &TeleopTurtle::odomGlobalGOTRawSLAMCallback, this);
  odom_global_got_corr_slam_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom_global_got_corr_slam", 10, &TeleopTurtle::odomGlobalGOTCorrSLAMCallback, this);

  //Subscribers for Client
  client_ground_truth_sub_ = nh_.subscribe<nav_msgs::Odometry>("/client/ground_truth", 10, &TeleopTurtle::client_groundTruthCallback, this);
  client_odom_local_sub_ = nh_.subscribe<nav_msgs::Odometry>("/client/odom_local", 10, &TeleopTurtle::client_odomLocalCallback, this);
  client_odom_global_got_raw_sub_ = nh_.subscribe<nav_msgs::Odometry>("/client/odom_global_got_raw", 10, &TeleopTurtle::client_odomGlobalGOTRawCallback, this);
  client_odom_global_got_corr_sub_ = nh_.subscribe<nav_msgs::Odometry>("/client/odom_global_got_corr", 10, &TeleopTurtle::client_odomGlobalGOTCorrCallback, this);
  client_got_raw_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/client/got_raw", 10, &TeleopTurtle::client_gotRaw, this);
  client_got_corr_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/client/got_corr", 10, &TeleopTurtle::client_gotCorr, this);


  service_ = nh_.advertiseService("get_pos_error", &TeleopTurtle::sendErr, this);

  // nh_.setParam("got_err_tolerance", err_tolerance_);
  // nh_.setParam("got_err_s", err_s_);
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

void TeleopTurtle::client_groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  client_ground_pos_.x = msg->pose.pose.position.x;
  client_ground_pos_.y = msg->pose.pose.position.y;
  client_ground_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::client_odomLocalCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  client_odom_local_pos_.x = msg->pose.pose.position.x;
  client_odom_local_pos_.y = msg->pose.pose.position.y;
  client_odom_local_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::client_odomGlobalGOTRawCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  client_odom_global_got_raw_pos_.x = msg->pose.pose.position.x;
  client_odom_global_got_raw_pos_.y = msg->pose.pose.position.y;
  client_odom_global_got_raw_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::client_odomGlobalGOTCorrCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  client_odom_global_got_corr_pos_.x = msg->pose.pose.position.x;
  client_odom_global_got_corr_pos_.y = msg->pose.pose.position.y;
  client_odom_global_got_corr_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::client_gotRaw(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  client_got_raw_pos_.x = msg->pose.pose.position.x;
  client_got_raw_pos_.y = msg->pose.pose.position.y;
  client_got_raw_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::client_gotCorr(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  client_got_corr_pos_.x = msg->pose.pose.position.x;
  client_got_corr_pos_.y = msg->pose.pose.position.y;
  client_got_corr_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::rawOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_raw_pos_.x = msg->pose.pose.position.x;
  odom_raw_pos_.y = msg->pose.pose.position.y;
  odom_raw_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::odomLocalCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_local_pos_.x = msg->pose.pose.position.x;
  odom_local_pos_.y = msg->pose.pose.position.y;
  odom_local_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::odomGlobalGOTRawCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_global_got_raw_pos_.x = msg->pose.pose.position.x;
  odom_global_got_raw_pos_.y = msg->pose.pose.position.y;
  odom_global_got_raw_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::odomGlobalGOTCorrCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_global_got_corr_pos_.x = msg->pose.pose.position.x;
  odom_global_got_corr_pos_.y = msg->pose.pose.position.y;
  odom_global_got_corr_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::slamCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  slam_pos_.x = msg->pose.position.x;
  slam_pos_.y = msg->pose.position.y;
  slam_pos_.z = 0;

  geometry_msgs::PoseWithCovarianceStamped slam_pose;
  //got_pose.header.seq = ++ frame_sequence_;
  slam_pose.header.stamp = ros::Time::now();
  slam_pose.header.frame_id = namespace_+"map";
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

void TeleopTurtle::odomGlobalSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_global_slam_pos_.x = msg->pose.pose.position.x;
  odom_global_slam_pos_.y = msg->pose.pose.position.y;
  odom_global_slam_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::odomGlobalGOTRawSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_global_got_raw_slam_pos_.x = msg->pose.pose.position.x;
  odom_global_got_raw_slam_pos_.y = msg->pose.pose.position.y;
  odom_global_got_raw_slam_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::odomGlobalGOTCorrSLAMCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_global_got_corr_slam_pos_.x = msg->pose.pose.position.x;
  odom_global_got_corr_slam_pos_.y = msg->pose.pose.position.y;
  odom_global_got_corr_slam_pos_.z = msg->pose.pose.position.z;
}

void TeleopTurtle::groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double ground_x = msg->pose.pose.position.x;
  double ground_y = msg->pose.pose.position.y;
  double ground_z = msg->pose.pose.position.z;

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

  // nh_.getParam("got_err_tolerance", err_tolerance_); // Error tolerance parameter, defaults to 0
  // nh_.getParam("got_err_s", err_s_); // Error tolerance parameter, defaults to 0

  ros::param::get("~got_err_tolerance", err_tolerance_);
  ros::param::get("~got_err_s", err_s_);

  // Error correction
  double dx = ground_x - got_x;
  double dy = ground_y - got_y;

  // double key_x = ((int)(ground_x * 1000)) / 1000.0;   // Store error for every 1 mm (0.001 m)
  // double key_y = ((int)(ground_y * 1000)) / 1000.0;
  float key_x = ((int)(ground_x * 10)) / 10.0;   // Store error for every 1 mm (0.001 m)
  float key_y = ((int)(ground_y * 10)) / 10.0;
  //ROS_INFO("Ground truth: ex:[%f] ey:[%f]", key_x, key_y);

  // Store error for every 0.5 meters
  // key_x = 0.5 * ((int) (key_x / 0.5));
  // key_y = 0.5 * ((int) (key_y / 0.5));

  Position pos = {key_x, key_y};

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

  // Store error
  Position err = {error_x, error_y};
  //ROS_INFO("Storing new error: ex:[%f] ey:[%f]", error_x, error_y);

  std::pair<Position, Position> p(pos, err);
  error_map_.insert(p);


  bool found_new_error = false;
  auto it = error_map_.find(pos);
  //if (error_map_.find(pos) != error_map_.end())
  if ( it != error_map_.end() )   // If an error is found
  {
      //ROS_INFO("CORRECTION: x:[%f] y[%f] ex:[%f] ey:[%f]", pos.x, pos.y, err.x, err.y);
      
      // Check is new error is bigger than the stored one
      if (fabs(error_x) > fabs(it->second.x))
      {
        error_x = it->second.x;
        found_new_error = true;
      }
      if (fabs(error_y) > fabs(it->second.y))
      {
        error_y = it->second.y;
        found_new_error = true;
      }

      // Replace stored error if a bigger one is found
      if(found_new_error)
      {
        error_map_.erase(it);
        Position err_new = {error_x, error_y};
        std::pair<Position, Position> p_new(pos, err_new);
        error_map_.insert(p_new);

        found_new_error = false;
        //ROS_INFO("NEW_ERROR: x:[%f] y[%f] ex:[%f] ey:[%f]", pos.x, pos.y, err_new.x, err_new.y);
      }
  }
  else  // If no error is found at that print an Error_msg
  {
    error_x = 0;
    error_y = 0;
    ROS_ERROR("Position not found");
  }

  // Publish got -> map TF
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  //transform.setOrigin( tf::Vector3((got_x + error_x), (got_y + error_y), 0.0) );
  transform.setOrigin( tf::Vector3((got_x), (got_y), 0.0) );
  tf::Quaternion quat;
  //q.setValue(0, 0, 0, 1);
  quat.setRPY(0,0,0);
  transform.setRotation(quat);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), namespace_+"got", namespace_+"map"));

  geometry_msgs::PoseWithCovarianceStamped got_pos;

  //current_pose.header.seq = ++ frame_sequence_;
  got_pos.header.stamp = ros::Time::now();
  got_pos.header.frame_id = namespace_+"got";
  got_pos.pose.pose.position.x = got_x;
  got_pos.pose.pose.position.y = got_y;
  got_pos.pose.pose.position.z = got_z;
  got_pos.pose.pose.orientation.x = quat.x();
  got_pos.pose.pose.orientation.y = quat.y();
  got_pos.pose.pose.orientation.z = quat.z();
  got_pos.pose.pose.orientation.w = quat.w();
  got_pos.pose.covariance[0] = 0.001;
  got_pos.pose.covariance[7] = 0.001;
  got_pos.pose.covariance[35] = 0.001;

  // Message for corrected got_pose
  geometry_msgs::PoseWithCovarianceStamped got_pos_corrected;

  //pose_corrected.header.seq = current_pose.header.seq;
  got_pos_corrected.header.stamp = ros::Time::now();
  got_pos_corrected.header.frame_id = namespace_+"got";
  got_pos_corrected.pose.pose.position.x = got_x + error_x;
  got_pos_corrected.pose.pose.position.y = got_y + error_y;
  got_pos_corrected.pose.pose.position.z = got_z;
  got_pos_corrected.pose.pose.orientation.x = quat.x();
  got_pos_corrected.pose.pose.orientation.y = quat.y();
  got_pos_corrected.pose.pose.orientation.z = quat.z();
  got_pos_corrected.pose.pose.orientation.w = quat.w();
  got_pos_corrected.pose.covariance[0] = 0.001;
  got_pos_corrected.pose.covariance[7] = 0.001;
  got_pos_corrected.pose.covariance[35] = 0.001;

  got_pub_.publish(got_pos);
  got_corrected_pub_.publish(got_pos_corrected);

 if(param_write_to_file_)
 {
    std::ofstream myfile;
    myfile.open ("/home/isircu/err_log.csv", std::ios::out | std::ios::app);

    myfile << ground_x << ','        // Ground truth (move_base goal position)
           << ground_y << ','
           << odom_raw_pos_.x << ','    //Raw odom
           << odom_raw_pos_.y << ','
           << odom_local_pos_.x << ','           // Odom+imu
           << odom_local_pos_.y << ','
           << odom_global_got_raw_pos_.x << ','  // Odom + imu + got_raw
           << odom_global_got_raw_pos_.y << ','
           << odom_global_got_corr_pos_.x << ','  // Odom + imu + got_corr
           << odom_global_got_corr_pos_.y << ','
           << slam_pos_.x << ','              // Slam raw error
           << slam_pos_.y << ','
           << odom_global_slam_pos_.x << ','      // SLAM +odom + imu -> EKF->
           << odom_global_slam_pos_.y << ','
           << odom_global_got_raw_slam_pos_.x << ','      // SLAM +odom + imu +got_raw -> EKF->
           << odom_global_got_raw_slam_pos_.y << ','
           << odom_global_got_corr_slam_pos_.x << ','      // SLAM +odom + imu +got_corr -> EKF->
           << odom_global_got_corr_slam_pos_.y << ','
           << got_x << ','                                  // Got raw error
           << got_y << ','
           << got_pos_corrected.pose.pose.position.x << ','           //Got corrected error
           << got_pos_corrected.pose.pose.position.y << ','
           << client_ground_pos_.x << ','    //Client values
           << client_ground_pos_.y << ','
           << client_odom_local_pos_.x << ','           // Odom+imu
           << client_odom_local_pos_.y << ','
           << client_odom_global_got_raw_pos_.x << ','  // Odom + imu + got_raw
           << client_odom_global_got_raw_pos_.y << ','
           << client_odom_global_got_corr_pos_.x << ','  // Odom + imu + got_corr
           << client_odom_global_got_corr_pos_.y << ','
           << client_got_raw_pos_.x << ','
           << client_got_raw_pos_.y << ','
           << client_got_corr_pos_.x << ','
           << client_got_corr_pos_.y << '\n';
    myfile.close();
 }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "master_got_position_node");
  TeleopTurtle teleop_turtle;

  ros::spin();
}