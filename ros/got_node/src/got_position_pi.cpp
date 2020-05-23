#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <bits/stdc++.h>

struct Position { 
	double x, y; 

	Position(double x_, double y_) 
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
		return (std::hash<double>()(p.x)) ^ 
			(std::hash<double>()(p.y)); 
	} 
}; 


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void teensyCallback(const geometry_msgs::Point::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  
  ros::NodeHandle nh_;

  ros::Publisher got_pub_;
  ros::Publisher error_pub_;
  ros::Subscriber teensy_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber amcl_sub_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> random_distribution_x_;
  std::normal_distribution<double> random_distribution_y_;
  std::normal_distribution<double> random_distribution_yaw_;
  unsigned frame_sequence_ = 0;
  unsigned frame_sequence_odom_ = 0;
  double offset_x_ = 0;
  double offset_y_ = 0;
  bool offset_obtained_ = false;
  double error_tolerance_ = 0; // 20 cm
  
  std::unordered_map<Position, Position, MyHashFunction> error_map_; 
  geometry_msgs::PoseWithCovarianceStamped odom_pose;
  geometry_msgs::Point error_msg;
   geometry_msgs::Point amcl_pos;

  Position pos = {0,0};
  Position err = {0,0};
  double err_x = 0;
  double err_y = 0;

};


TeleopTurtle::TeleopTurtle()
{
  //nh_.param("~got_error_tolerance", error_tolerance_, 0); // Error tolerance parameter, defaults to 0
  got_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_pose2", 1);
  error_pub_ = nh_.advertise<geometry_msgs::Point>("got_pose_error", 1);
  teensy_sub_ = nh_.subscribe<geometry_msgs::Point>("got_teensy", 10, &TeleopTurtle::teensyCallback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &TeleopTurtle::odomCallback, this);
  amcl_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, &TeleopTurtle::amclCallback, this);
}

void TeleopTurtle::teensyCallback(const geometry_msgs::Point::ConstPtr& msg)
{

  if (!offset_obtained_)
  {
    //if (odom_pose.pose.pose.position.x != 0) // Wait to get actual data as it takes a while to get a reading sometimes
    if (msg->x != 0) // Wait to get actual data as it takes a while to get a reading sometimes
    {
      // offset_x_ = odom_pose.pose.pose.position.x - msg->x;
      // offset_y_ = odom_pose.pose.pose.position.y - msg->y;
      offset_x_ = amcl_pos.x - msg->x;
      offset_y_ = amcl_pos.y - msg->y;
      offset_obtained_ = true;
    }
  } else
  {
    // error_msg.x = odom_pose.pose.pose.position.x - msg->x;
    // error_msg.y = odom_pose.pose.pose.position.y - msg->y;
    error_msg.x = amcl_pos.x - msg->x;
    error_msg.y = amcl_pos.y - msg->y;
    error_msg.z = 0;



    if ((abs(error_msg.x) > error_tolerance_) || (abs(error_msg.y) > error_tolerance_))
    {
      //pos = {odom_pose.pose.pose.position.x, odom_pose.pose.pose.position.y};
      pos = {amcl_pos.x, amcl_pos.y};
      err = {error_msg.x, error_msg.y};

      std::pair<Position, Position> p(pos, err);
      error_map_.insert(p);
    }

    err_x = 0;
    err_y = 0;
    //pos = {odom_pose.pose.pose.position.x, odom_pose.pose.pose.position.y};
    pos = {amcl_pos.x, amcl_pos.y};

    // Check if there's an error stored at current position
    if (error_map_.find(pos) != error_map_.end())
    {
        auto it = error_map_.find(pos);

        err_x = it->second.x;
        err_y = it->second.y;
        ROS_INFO("Got error X:%f, Y:%f", err_x, err_y);
    }
    

    geometry_msgs::PoseWithCovarianceStamped got_pose;
    got_pose.header.seq = ++ frame_sequence_;
    got_pose.header.stamp = ros::Time::now();
    got_pose.header.frame_id = "odom";
    //got_pose.pose.pose.position.x = msg->pose.pose.position.x;
    //got_pose.pose.pose.position.x = msg->x + offset_x_ + error_x;
    // got_pose.pose.pose.position.x = msg->x + offset_x_ + error_x - error_x_old;
    // got_pose.pose.pose.position.y = msg->y + offset_y_ + error_y - error_x_old;
    got_pose.pose.pose.position.x = msg->x + err_x;
    got_pose.pose.pose.position.y = msg->y + err_y;
    got_pose.pose.pose.position.z = msg->z;
    got_pose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0., 0., 0.);
    got_pose.pose.covariance = boost::array<double, 36>({
        std::pow(random_distribution_x_.mean() + random_distribution_x_.stddev(), 2), 0., 0., 0., 0., 0.,
        0., std::pow(random_distribution_y_.mean() + random_distribution_y_.stddev(), 2), 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., std::pow(random_distribution_yaw_.mean() + random_distribution_yaw_.stddev(), 2)});

    got_pub_.publish(got_pose);
    error_pub_.publish(error_msg);
  }

}

void TeleopTurtle::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //geometry_msgs::PoseWithCovarianceStamped odom_pose;
  odom_pose.header.seq = ++ frame_sequence_odom_;
  odom_pose.header.stamp = ros::Time::now();
  odom_pose.header.frame_id = "odom";
  odom_pose.pose.pose.position.x = msg->pose.pose.position.x;
  odom_pose.pose.pose.position.y = msg->pose.pose.position.y;
  odom_pose.pose.pose.position.z = msg->pose.pose.position.z;
  odom_pose.pose.pose.orientation = msg->pose.pose.orientation;
  odom_pose.pose.covariance = boost::array<double, 36>({
      std::pow(random_distribution_x_.mean() + random_distribution_x_.stddev(), 2), 0., 0., 0., 0., 0.,
      0., std::pow(random_distribution_y_.mean() + random_distribution_y_.stddev(), 2), 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., std::pow(random_distribution_yaw_.mean() + random_distribution_yaw_.stddev(), 2)});
}

void TeleopTurtle::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  amcl_pos.x = msg->pose.pose.position.x;
  amcl_pos.y = msg->pose.pose.position.y;
  amcl_pos.z = 0;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "got_position_node");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
