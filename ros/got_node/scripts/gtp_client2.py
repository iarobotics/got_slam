#! /usr/bin/env python

# import ros stuff
import rospy
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

import time
import math

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0

dict_state_ = {0:"rotating to face the goal", 1:"going straight", 2:"Goal reached"}
# goal
desired_position_ = Point()
#desired_position_.x = rospy.get_param('des_pos_x')
#desired_position_.y = rospy.get_param('des_pos_y')
#desired_position_.z = 0
# parameters
#yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
yaw_precision_ = math.pi / 30 # +/- 2 degree allowed

dist_precision_ = 0.1

linVel = 0.3
angVel = 0.7

desired_position_old_ = Point()
#desired_position_old_.x = desired_position_.x
#desired_position_old_.y = desired_position_.y
#desired_position_old_.z = desired_position_.z

# publishers
pub = None

# (x,y) position goal. Robot starts (2.5, -4.0)
goal_points_ = [
    (0.0, 0.0),
    (0.0, 3.0),
    (2.0, 3.0),
    (4.0, 4.0),
    (6.0, 3.0),
    (6.0, -2.0),
    (5.0, -3.0)
]

rospy.set_param('des_pos_x', goal_points_[0][0])
rospy.set_param('des_pos_y', goal_points_[0][1])
pos_index_ = 1


# Callback  - update robot position and yaw (heading)
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    global state_
    state_ = state
    #print("State changed to [{}]: {}".format(state, dict_state_[state]))
    if state == 2:
        print("Goal reached")


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


# Rotate the robot around its axis to face the goal
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    # rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = angVel if err_yaw > 0 else -angVel

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        # print 'Yaw error: [%s]' % err_yaw
        change_state(1)


# Move towards the goal in a straigth line
def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = linVel
        #twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        twist_msg.angular.z = angVel if err_yaw > 0 else -angVel
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        # print 'Yaw error: [%s]' % err_yaw
        change_state(0)


def done():
    global pos_index_

    if pos_index_ != len(goal_points_):
        goal_x = goal_points_[pos_index_][0]
        goal_y = goal_points_[pos_index_][1]
        rospy.set_param('des_pos_x', goal_x)
        rospy.set_param('des_pos_y', goal_y)
        print("Set destination to x: {}, y: {}".format(goal_x, goal_y))
        pos_index_ += 1

    else:
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)


def get_goal_position():
    global desired_position_, desired_position_old_

    desired_position_.x = rospy.get_param('des_pos_x')
    desired_position_.y = rospy.get_param('des_pos_y')

    if (desired_position_.x != desired_position_old_.x) or (desired_position_.y != desired_position_old_.y):
        desired_position_old_.x = desired_position_.x
        desired_position_old_.y = desired_position_.y
        change_state(0)


def main():
    global pub

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('raw_odom', Odometry, clbk_odom)

    rate = rospy.Rate(200)

    while not rospy.is_shutdown():

        get_goal_position()

        if state_ == 0:
            fix_yaw(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            done()
            pass
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()


if __name__ == '__main__':
    main()