#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>

#define ESTIMATE_POSITION

#define NUM_BEACONS 5

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

//   ros::service::waitForService("spawn");
//   ros::ServiceClient add_turtle =
//     node.serviceClient<turtlesim::Spawn>("spawn");
//   turtlesim::Spawn srv;
//   add_turtle.call(srv);

//   ros::Publisher turtle_vel =
//     node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10);

  tf::TransformListener listener;

  double x_est=0, y_est=0, z_est=0;
  // TODO get from ROS parameters
  int beacon_locations[NUM_BEACONS][3] = { {-5,-5,5}, {5,-5,5}, {0,0,5}, {-5,5,5}, {5,5,5}};
  //int beacon_locations[5][3]={{1645,-462,4500},{1591,4193,4500},{1591,4193,3999},{6195,0,5499},{10157,4204,5499}};

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform b1_transform;
    tf::StampedTransform b2_transform;
    tf::StampedTransform b3_transform;
    tf::StampedTransform b4_transform;
    tf::StampedTransform b5_transform;

    tf:: StampedTransform tfList[NUM_BEACONS] = { b1_transform, b2_transform, b3_transform, b4_transform, b5_transform };

    try
    {
        // \brief Get the transform between two frames by frame ID.
        // \param target_frame The frame to which data should be transformed
        //\param source_frame The frame where the data originated
      listener.lookupTransform("/base_link", "/beacon1", ros::Time(0), b1_transform);
      listener.lookupTransform("/base_link", "/beacon2", ros::Time(0), b2_transform);
      listener.lookupTransform("/base_link", "/beacon3", ros::Time(0), b3_transform);
      listener.lookupTransform("/base_link", "/beacon4", ros::Time(0), b4_transform);
      listener.lookupTransform("/base_link", "/beacon5", ros::Time(0), b5_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // turtlesim::Velocity vel_msg;
    // vel_msg.angular = atan2(transform.getOrigin().y(), transform.getOrigin().x());

    // vel_msg.linear = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    double b1_distance = sqrt(pow(b1_transform.getOrigin().x(), 2) + pow(b1_transform.getOrigin().y(), 2));
    double b2_distance = sqrt(pow(b2_transform.getOrigin().x(), 2) + pow(b2_transform.getOrigin().y(), 2));
    double b3_distance = sqrt(pow(b3_transform.getOrigin().x(), 2) + pow(b3_transform.getOrigin().y(), 2));
    double b4_distance = sqrt(pow(b4_transform.getOrigin().x(), 2) + pow(b4_transform.getOrigin().y(), 2));
    double b5_distance = sqrt(pow(b5_transform.getOrigin().x(), 2) + pow(b5_transform.getOrigin().y(), 2));

    double beacon_distances[NUM_BEACONS] = {b1_distance, b2_distance, b3_distance, b4_distance, b5_distance};

    //turtle_vel.publish(vel_msg);


    #ifdef ESTIMATE_POSITION
            //TODO: Jump over if out of range of the beacon ~11 meters

        for(uint8_t index = 0; index < NUM_BEACONS+1; index++)
        {
            double dp=(double)pow(x_est-(double)beacon_locations[index][0],2);
            dp+=(double)pow(y_est-(double)beacon_locations[index][1],2);
            dp+=(double)pow(z_est-(double)beacon_locations[index][2],2);
            dp=sqrt(dp);
            ROS_INFO("DP: %f", dp);

            //meas_dist*=0.343; //Speed of light in mm pr uS
            //if(meas_dist>11000 || meas_dist<1000)

            //double lambda = 1 - dp / (beacon_distances[index] * 10000);
            double lambda = 1 - dp / 10000;
            //ROS_INFO("Lambda: %f", lambda);
            // ROS_INFO("Distance: %d, %f;", index, beacon_distances[index]);
            // ROS_INFO("Location: %d, %f, %f, %f;", index, beacon_locations[index][0], beacon_locations[index][1], beacon_locations[index][2]);

            double x_est_new=x_est/(1-lambda)-(double)beacon_locations[index][0]*lambda/(1-lambda);
            double y_est_new=y_est/(1-lambda)-(double)beacon_locations[index][1]*lambda/(1-lambda);
            double z_est_new=z_est/(1-lambda)-(double)beacon_locations[index][2]*lambda/(1-lambda);

            x_est = x_est_new;
            y_est = y_est_new;
            z_est = z_est_new;
        }

    #endif

    //ROS_INFO("Distance: %f, %f, %f, %f, %f;", b1_distance, b2_distance, b3_distance, b4_distance, b5_distance);
    ROS_INFO("Position: %f, %f, %f;", x_est, y_est, z_est);

    rate.sleep();
  }
  return 0;
};

//sqrt( (x1 -x2)² + (y1-y2)² )
