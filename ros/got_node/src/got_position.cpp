//#define __robot_localization_demo__positioning_system__

#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <geometry_msgs/Pose.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <random>

#define ESTIMATE_POSITION

#define NUM_BEACONS 5

  std::default_random_engine random_generator_;
  std::normal_distribution<double> random_distribution_x_;
  std::normal_distribution<double> random_distribution_y_;
  std::normal_distribution<double> random_distribution_yaw_;

int main(int argc, char** argv){
  ros::init(argc, argv, "got_position");

  ros::NodeHandle node;

  tf::TransformListener listener;

  //ros::Publisher pose_pub = node.advertise<geometry_msgs::Pose>("got_pose", 100);
  ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("got_pose", 100);

  double x_est=0, y_est=0, z_est=0;
  // TODO get from ROS parameters
  int beacon_locations[NUM_BEACONS][3] = { {-5,-5,5}, {5,-5,5}, {0,0,5}, {-5,5,5}, {5,5,5}};
  char frames[5][10] = {"beacon1", "beacon2", "beacon3", "beacon4", "beacon5"};

  unsigned frame_sequence_{0};

  //int beacon_locations[5][3]={{1645,-462,4500},{1591,4193,4500},{1591,4193,3999},{6195,0,5499},{10157,4204,5499}};

  ros::Rate rate(100.0);
  while (node.ok())
  {
    //geometry_msgs::Pose pose_msg;

    for(uint8_t index = 0; index < NUM_BEACONS; index++)
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/base_link", frames[index], ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.01).sleep();
        }

        // Actual distance in meters from beacon to robot
        double distance = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2) + pow(transform.getOrigin().z(), 2));

        if(distance>10 || distance<1)
            continue;

        // Distance to estimated cartesian coordinates
        double dp=(double)pow(x_est-(double)beacon_locations[index][0], 2);
        dp+=(double)pow(y_est-(double)beacon_locations[index][1], 2);
        dp+=(double)pow(z_est-(double)beacon_locations[index][2], 2);
        dp=sqrt(dp);
        //ROS_INFO("DP: %f", dp);

        // Normalizing factor
        double lambda = 1 - dp / distance;
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
    // pose_msg.position.x = x_est;
    // pose_msg.position.y = y_est;
    // pose_msg.position.z = z_est;

    // pose_pub.publish(pose_msg);
    //ROS_INFO("Post_Position: %f, %f, %f;", x_est, y_est, z_est);

//////////
    // measurement.x += random_distribution_x_(random_generator_);
    // measurement.y += random_distribution_y_(random_generator_);
    // measurement.theta += random_distribution_yaw_(random_generator_);
    // Publish measurement.
    geometry_msgs::PoseWithCovarianceStamped current_pose;
    current_pose.header.seq = ++ frame_sequence_;
    current_pose.header.stamp = ros::Time::now();
    //current_pose.header.frame_id = "map";
    current_pose.header.frame_id = "odom";
    //current_pose.pose.pose.position.x = x_est + random_distribution_x_(random_generator_);
    //current_pose.pose.pose.position.y = y_est + random_distribution_y_(random_generator_);
    current_pose.pose.pose.position.x = x_est;
    current_pose.pose.pose.position.y = y_est;
    current_pose.pose.pose.position.z = 0.;
    current_pose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0., 0., 0.);
    current_pose.pose.covariance = boost::array<double, 36>({
        std::pow(random_distribution_x_.mean() + random_distribution_x_.stddev(), 2), 0., 0., 0., 0., 0.,
        0., std::pow(random_distribution_y_.mean() + random_distribution_y_.stddev(), 2), 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., std::pow(random_distribution_yaw_.mean() + random_distribution_yaw_.stddev(), 2)});
    pose_pub.publish(current_pose);

///////////
    rate.sleep();
  }
  return 0;

};

