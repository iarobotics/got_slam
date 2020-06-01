#include "ros/ros.h"
#include "got_node/AddTwoInts.h"
#include "got_node/GetPosError.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<got_node::GetPosError>("get_pos_error");
  got_node::GetPosError srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Server response: %f, %f", srv.response.err_x, srv.response.err_y);
  }
  else
  {
    ROS_ERROR("Failed to call service get_pos_error");
    return 1;
  }

  return 0;
}