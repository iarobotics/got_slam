#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/*int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;


  ros::Rate rate(1000.0);
  while(node.ok()){
        transform.setOrigin(  tf::Vector3(5, 0.0, 0.0) );
        transform.setRotation(tf::Quaternion(0, 0, 0, 1)       );

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/odom", "/beacon1"));
        rate.sleep();
  }

  return 0;
};*/


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  static tf::TransformBroadcaster br;
  tf::Transform beacon1, beacon2, beacon3, beacon4, beacon5;

  beacon1.setOrigin( tf::Vector3(-5, 0, 5));
  beacon1.setRotation(tf::Quaternion(0, 0, 0, 1));

  beacon2.setOrigin( tf::Vector3(5, 0, 5));
  beacon2.setRotation(tf::Quaternion(0, 0, 0, 1));

  beacon3.setOrigin( tf::Vector3(0, 5, 5));
  beacon3.setRotation(tf::Quaternion(0, 0, 0, 1));

  beacon4.setOrigin( tf::Vector3(-5, 10, 5));
  beacon4.setRotation(tf::Quaternion(0, 0, 0, 1));
  
  beacon5.setOrigin( tf::Vector3(5, 10, 5));
  beacon5.setRotation(tf::Quaternion(0, 0, 0, 1));

/*  const std::vector< tf::StampedTransform > beacons = {
    tf::StampedTransform(beacon1, ros::Time::now(), "/odom", "/beacon1"),
    tf::StampedTransform(beacon2, ros::Time::now(), "/odom", "/beacon2"),
    tf::StampedTransform(beacon3, ros::Time::now(), "/odom", "/beacon3"),
    tf::StampedTransform(beacon4, ros::Time::now(), "/odom", "/beacon4"),
    tf::StampedTransform(beacon5, ros::Time::now(), "/odom", "/beacon5")
  };*/


  ros::Rate rate(1000.0);
  while(node.ok()){

        br.sendTransform(tf::StampedTransform(beacon1, ros::Time::now(), "/odom", "/beacon1"));
        br.sendTransform(tf::StampedTransform(beacon2, ros::Time::now(), "/odom", "/beacon2"));
        br.sendTransform(tf::StampedTransform(beacon3, ros::Time::now(), "/odom", "/beacon3"));
        br.sendTransform(tf::StampedTransform(beacon4, ros::Time::now(), "/odom", "/beacon4"));
        br.sendTransform(tf::StampedTransform(beacon5, ros::Time::now(), "/odom", "/beacon5"));

        rate.sleep();
  }

  return 0;
};

/*std::vector<int> v = {7, 5, 16, 8};

static tf::TransformBroadcaster br;
        br.sendTransform( tf::StampedTransform(tf, ros::Time::now(), "frame_1", "frame_2") );
        br.sendTransform( tf::StampedTransform(tf, ros::Time::now(), "frame_2", "frame_3") );

void TransformBroadcaster::sendTransform(const StampedTransform & transform)

void tf::TransformBroadcaster::sendTransform  (   const std::vector< StampedTransform > &   transforms  )   */