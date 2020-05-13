#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::Point goal;


void callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

  goal.x = msg->pose.position.x;
  goal.y = msg->pose.position.y;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "move_base_goal_publisher");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/move_base_simple/goal", 10, &callback);

  ros::Publisher pub = node.advertise<geometry_msgs::Point>("goal_position_mb", 10);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    pub.publish(goal);
    ros::spinOnce();
  }
  return 0;
};