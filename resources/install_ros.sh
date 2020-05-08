#! /bin/bash

## InstalL ros_kinetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update

sudo apt-get -y install ros-kinetic-desktop-full

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get -y git install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo rosdep init
rosdep update


sudo apt-get -y install ros-kinetic-map-server ros-kinetic-amcl ros-kinetic-move-base ros-kinetic-navigation
sudo apt-get -y install ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-hector-mapping

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

git clone https://github.com/iarobotics/got_slam.git
cd ~/catkin_ws && catkin_make


echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Setup complete..."

## Launch the simulation
#roslaunch got_node slam.launch

## Launch go_to_point
#rosrun got_node go_to_point.py

## Use keyboard teleoperation
#roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

## Save the map
#rosrun map_server map_saver -f ~/map

```
