#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#define NUM_BEACONS 5
int beacon_locations[NUM_BEACONS][3] = { {-5,-5,5}, {5,-5,5}, {0,0,5}, {-5,5,5}, {5,5,5}};
double x_est=0, y_est=0, z_est=0;


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void odomCallback(const geometry_msgs::Point::ConstPtr& msg);
  
  ros::NodeHandle nh_;

  ros::Publisher odom_pub_;
  ros::Subscriber odom_sub_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> random_distribution_x_;
  std::normal_distribution<double> random_distribution_y_;
  std::normal_distribution<double> random_distribution_yaw_;
  unsigned frame_sequence_;
  
};


TeleopTurtle::TeleopTurtle()
{
  odom_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_pose", 1);

  odom_sub_ = nh_.subscribe<geometry_msgs::Point>("teensy_got", 10, &TeleopTurtle::odomCallback, this);
}

void TeleopTurtle::odomCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  geometry_msgs::PoseWithCovarianceStamped current_pose;
  current_pose.header.seq = ++ frame_sequence_;
  current_pose.header.stamp = ros::Time::now();
  current_pose.header.frame_id = "odom";
  //current_pose.pose.pose.position.x = msg->pose.pose.position.x;
  current_pose.pose.pose.position.x = msg->x;
  current_pose.pose.pose.position.y = msg->y;
  current_pose.pose.pose.position.z = msg->z;
  current_pose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0., 0., 0.);
  current_pose.pose.covariance = boost::array<double, 36>({
      std::pow(random_distribution_x_.mean() + random_distribution_x_.stddev(), 2), 0., 0., 0., 0., 0.,
      0., std::pow(random_distribution_y_.mean() + random_distribution_y_.stddev(), 2), 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., std::pow(random_distribution_yaw_.mean() + random_distribution_yaw_.stddev(), 2)});

  odom_pub_.publish(current_pose);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "got_position_node");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
