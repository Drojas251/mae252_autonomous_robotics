# MAE 252 Autonomous Robotics

## Prerequisites
  - ROS Noetic Desktop
  - Gazebo 11.XX
  - gazebo_ros_pkgs
  
## Install
Once you have a base install of ROS Noetic and Gazebo11, run these commands in a terminal to source ROS set up files. This command will add the ROS source setup to your .bashrc which will source the setup files everytime you open a new terminal. 

```
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
Next make sure you have gazebo_ros_pkgs installed

```
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```
Now you are ready to pull this repo. Clone it to your home directory 

```
git clone https://github.com/Drojas251/mae252_autonomous_robotics.git
```
Inside this repo is a bash script that will automate the workspace setup. This will setup your catkin workspace and build it. Run these two commands 

```
chmod +x install.sh
./install.sh
```


After this script is ran, your catkin workspace will look as such:

### catkin workspace
  - build
  - devel
  - src
    - mae252_autonomous_robotics
    - turtlebot3
    - turtlebot3_msgs
    - turtlebot3_simulation
	

You need to then source your workspace setup files. Make sure you are in the autonomous_robotics directory. Run these commands 

```
cd ~/autonomous_robotics
source devel/setup.bash
```

Now you are ready to run code!

### DEMO
Go into the autonomous_robotics directory and run the demo

```
cd ~/autonomous_robotics
roslaunch robot_gazebo robot_world.launch
```

### Other Things to Install

```
sudo apt-get install ros-noetic-ros-numpy
```

