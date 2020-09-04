#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

import serial
import struct


def move():

    linVel = 0.3;
    angVel = 0.7;

    portName = "/dev/ttyUSB0"
    baudRate = 115200

    # Starts a new node
    rospy.init_node('armband_node')
    #velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
    euler_publisher = rospy.Publisher('euler_deg', Point, queue_size=1)
    euler_msg = Point()
    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y= 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y= 0
    vel_msg.angular.z = 0


    #numFSR = 20              # number of FSR sensor values
    numFSR = 3              # number of FSR sensor values

    # dataNumBytes = 1        # number of bytes of 1 data point
    # dataType = 'B'     # 1 byte unsigned char

    dataNumBytes = 4        # number of bytes of 1 data point
    dataType = 'f'     # 4 byte float

    loops = 0
    
    print('Trying to connect to: ' + str(portName) + ' at ' + str(baudRate) + ' BAUD.')
    try:
        s = serial.Serial(portName, baudRate, timeout=4)
        print('Connected to ' + str(portName) + ' at ' + str(baudRate) + ' BAUD.')
        s.write(str.encode('S'))
    except:
        print("Failed to connect with " + str(portName) + ' at ' + str(baudRate) + ' BAUD.')
        exit()

    # print('Trying to connect to: ' + str(portName) + ' at ' + str(baudRate) + ' BAUD.')

    # s = serial.Serial(portName, baudRate, timeout=4)
    # print('Connected to ' + str(portName) + ' at ' + str(baudRate) + ' BAUD.')
    # s.write(str.encode('S'))

    while not rospy.is_shutdown():
        try:
            loops += 1
            data = s.read(numFSR*dataNumBytes)

            if(data is None):
                continue

            data_decoded = list()

            for i in range(numFSR):
                temp = data[(i * dataNumBytes):(dataNumBytes + (i * dataNumBytes))]
                value, = struct.unpack(dataType, temp)
                data_decoded.append(value)

            # print(data_decoded)

            roll = data_decoded[2]
            pitch = data_decoded[1]
            yaw = data_decoded[0]
            
            euler_msg.x = roll
            euler_msg.y = pitch
            euler_msg.z = yaw


            if pitch > 60:
                vel_msg.linear.x = linVel
            elif pitch < -60:
                vel_msg.linear.x = -linVel
            else:
                vel_msg.linear.x = 0

            if roll > 60:
                vel_msg.angular.z = angVel
            elif roll < -60:
                vel_msg.angular.z = -angVel
            else:
                vel_msg.angular.z = 0

            velocity_publisher.publish(vel_msg)
            euler_publisher.publish(euler_msg)

        except KeyboardInterrupt:
            print("Exiting...")
            break
    s.write(str.encode('T'))
    #s.close()


if __name__ == '__main__':
    move()
