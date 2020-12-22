
#include <ros.h>
#include <geometry_msgs/Point.h>
#include "got_serial.h"



ros::NodeHandle nh;

geometry_msgs::Point msg;
ros::Publisher pub("got_pose", &msg);

void setup() {

  nh.initNode();
  nh.advertise(pub);

  delay(3000);
  //Serial.begin(115200);

  Serial3.begin(115200); // Required for GOT position data

    // while(1==0)
    // {
    //   for(int i=0;i<21;i++)
    //    Serial.write(test_bytes[i]);
    //    delay(100);
    // }

}

void loop() {
  Get_Position(); //double x_est,y_est,z_est defined in got_serial.h
  // print_got_pos();
  // delay(100);
  msg.x = x_est;
  msg.y = y_est;
  msg.z = z_est;
  pub.publish( &msg );
  nh.spinOnce();
  delay(10);

}

// void print_got_pos() {
//    Serial.print("x_got: ");
//    Serial.print(x_est);
//    Serial.print("\t y_got: ");
//    Serial.print(y_est);
//    Serial.print("\t z_got: ");
//    Serial.println(z_est);

// }
