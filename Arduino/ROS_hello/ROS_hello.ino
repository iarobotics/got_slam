/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
//#define USE_USBCON

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

#define ROS_TELEOP

#define LED_PIN 12  //Green led for YAMC board


#ifdef ROS_TELEOP
  boolean move_robot = false;

  void teleopCb(const geometry_msgs::Twist& msg)
  {
    digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);               // wait for a second
    digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);               // wait for a second
    
    Serial.print(msg.linear.x);
    Serial.print(', ');
    Serial.print(msg.linear.y);
    Serial.print(', ');
    Serial.print(msg.linear.z);
    Serial.print('; ');

    Serial.print(msg.angular.x);
    Serial.print(', ');
    Serial.print(msg.angular.y);
    Serial.print(', ');
    Serial.println(msg.angular.z);

  }

  ros::Subscriber<geometry_msgs::Twist> teleop_sub("turtle1/cmd_vel", &teleopCb);

#endif

char hello[13] = "hello world!";

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(chatter);

  #ifdef ROS_TELEOP
    nh.subscribe(teleop_sub);
  #endif
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
