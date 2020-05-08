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
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
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
  // nh_.param("axis_linear", linear_, linear_);
  // nh_.param("axis_angular", angular_, angular_);
  // nh_.param("scale_angular", a_scale_, a_scale_);
  // nh_.param("scale_linear", l_scale_, l_scale_);


  //odom_pub_ = nh_.advertise<geometry_msgs::Pose>("got_pose2", 1);
  odom_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_pose", 1);


  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("ground_truth", 10, &TeleopTurtle::odomCallback, this);

}

void TeleopTurtle::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

  for(uint8_t index = 0; index < NUM_BEACONS; index++)
  {
    // Actual distance in meters from beacon to robot
    double dist=(double)pow(msg->pose.pose.position.x-(double)beacon_locations[index][0], 2);
    dist+=(double)pow(msg->pose.pose.position.y-(double)beacon_locations[index][1], 2);
    dist+=(double)pow(msg->pose.pose.position.z-(double)beacon_locations[index][2], 2);
    dist=sqrt(dist);


    if ((msg->pose.pose.position.x < 3) && (msg->pose.pose.position.x > 1) && (msg->pose.pose.position.y > -1) && (msg->pose.pose.position.y < 1) && (index == 2))
    {
      dist += 0.2;
    }

    if(dist>10 || dist<1)
        continue;

    // Distance to estimated cartesian coordinates
    double dp=(double)pow(x_est-(double)beacon_locations[index][0], 2);
    dp+=(double)pow(y_est-(double)beacon_locations[index][1], 2);
    dp+=(double)pow(z_est-(double)beacon_locations[index][2], 2);
    dp=sqrt(dp);
    //ROS_INFO("DP: %f", dp);

    // Normalizing factor
    double lambda = 1 - dp / dist;
    //ROS_INFO("Lambda: %f", lambda);

    // Estimated Cartesian coordinates
    double x_est_new=x_est/(1-lambda)-(double)beacon_locations[index][0]*lambda/(1-lambda);
    double y_est_new=y_est/(1-lambda)-(double)beacon_locations[index][1]*lambda/(1-lambda);
    double z_est_new=z_est/(1-lambda)-(double)beacon_locations[index][2]*lambda/(1-lambda);

    x_est = x_est_new;
    y_est = y_est_new;
    z_est = z_est_new;
    //ROS_INFO("Position: %d, %f, %f, %f;", index, x_est, y_est, z_est);
  }

  geometry_msgs::PoseWithCovarianceStamped current_pose;
  current_pose.header.seq = ++ frame_sequence_;
  current_pose.header.stamp = ros::Time::now();
  current_pose.header.frame_id = "odom";
  //current_pose.pose.pose.position.x = msg->pose.pose.position.x;
  current_pose.pose.pose.position.x = x_est;
  current_pose.pose.pose.position.y = y_est;
  current_pose.pose.pose.position.z = z_est;
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
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
