#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import TwistStamped

def talker():
    pub = rospy.Publisher('cmd_vel', TwistStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        vel = TwistStamped()
        vel.header.stamp = rospy.Time.now()
        vel.header.frame_id = "odom"
        vel.twist.linear.x = 0.5

        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
