#!/usr/bin/env bash

export catkin_ws=~/autonomous_robotics
export repo=mae252_autonomous_robotics
cd ..
mkdir -p $catkin_ws/src

mv ~/$repo $catkin_ws/src

cd ~

sudo apt-get update
sudo apt-get upgrade

cd $catkin_ws/src

git clone -b noetic-devel https://github.com/Drojas251/turtlebot3.git
git clone -b noetic-devel https://github.com/Drojas251/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/Drojas251/turtlebot3_simulations.git
git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git

cd $catkin_ws
catkin_make
