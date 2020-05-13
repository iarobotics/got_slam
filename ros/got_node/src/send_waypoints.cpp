#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <array>
#include <geometry_msgs/Point.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// int x [5] = { 1, 2, 2, 4, 4}; 
// int y [5] = {0, 0 ,2 , 2, 4};

// int x [10] = {-1, -2, -2, 1, 2, 1, -2, -2, -1, 0}; 
// int y [10] = {0, 1, 4, 3, 4, 3, 4, 1, 0, 0};

int x [4] = {-1, -2, -2, 1}; 
int y [4] = {0, 1, 4, 3};


int lenght0f_Array = sizeof(x) / sizeof(x[0]);


int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Point>("move_base_goal", 10);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_footprint";
    goal.target_pose.header.stamp = ros::Time::now();
    for( int i = 0; i < 4; i++ ) 
    {
        // goal.target_pose.pose.position.x = x[i];
        // goal.target_pose.pose.orientation.w = y[i];
        goal.target_pose.pose.position.x = x[i];
        goal.target_pose.pose.position.y = y[i];
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal");

        geometry_msgs::Point goal_position;
        goal_position.x = x[i];
        goal_position.y = y[i];
        goal_position.z = 0;
        pub.publish(goal_position);


        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
        else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
        }


    return 0;
}