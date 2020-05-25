# AUTONOMOUS VEHICLE CONTROL AND BEHAVIOUR
### Ngee Ann Polytechnic Engineering Science Final Year Project with MooVita

# Abstract
This project covers the development of an autonomous vehicle platform in a simulated environment using ROS and Gazebo implementing a think-sense-act cycle in navigating in a virtual world, avoiding static and moving objects.

## Requirements
### Operating System
1. [Ubuntu 16.04.6 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/)

### Software
1. [Desktop-Full ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
   - [map_server](http://wiki.ros.org/map_server)
   - [fake_localization](http://wiki.ros.org/fake_localization)
   - [joint_state_publisher](http://wiki.ros.org/joint_state_publisher)
   - joint_state_publisher_gui
   - [ros_controller](http://wiki.ros.org/ros_control#Install)
	 
2. [Python 2.7](https://www.python.org/download/releases/2.7/)
   - [pip](https://pypi.org/project/pip/)
   - [NumPy](https://pypi.org/project/numpy/) (Not required if you do not plan to use circle_road_gen.py)
  
3. [Gazebo 7.1](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install)
   - [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

4. [Git](https://git-scm.com/download/linux)

## Installation
1. Install [Ubuntu 16.04.6 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/)

2. Install [Desktop-Full ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
   - Follow instructions
   - Install everything else on the page
   
3. Install [NumPy](https://pypi.org/project/numpy/)
   - `pip install NumPy`
   
### If you have none of the required packages
1. Change directory to your cloned path
   - Go to your terminal
   - Type `cd catkin_ws/src/fyp-moovita

2. Type `./requirements` to install the required packages

### If you already have one of the required packages
1. Update package list
   - `sudo apt-get update`
 
2. Install [Gazebo 7.1](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install)
   - `sudo apt-get install gazebo7`
   - `sudo apt-get install libgazebo7-dev`
  
3. Install [Git](https://git-scm.com/download/linux)
   - `sudo apt-get install git`
  
4. Install [fake_localization](http://wiki.ros.org/fake_localization)
   - `sudo apt-get install ros-kinetic-fake-localization`
  
5. Install [joint_state_publisher](http://wiki.ros.org/joint_state_publisher)
    - `sudo apt-get install joint-state-publisher`
  
6. Install joint_state_publisher_gui
    - `sudo apt-get install ros-kinetic-joint-state-publisher-gui`
  
7. Install [ros_controller](http://wiki.ros.org/ros_control#Install)
    - `sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers`
    
8. Install [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
    - `sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control`
    
## Usage
1. Create catkin workspace
   - Open Terminal
   - Type `cd`
   - Type `mkdir -p catkin_ws/src`
   - Type `cd catkin_ws`
   
2. Place ROS package into source folder
   - Type `nautilus ~/catkin_ws/src`
   - Drag and drop ngeeann_av_description into folder
   
3. Setup ROS workspace
   - Type `source devel/setup.bash`
   - Type `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
   
## circle_road_gen.py 
### Description
circle_road_gen.py is a custom script which will generate the <point> coordinates of a circle for Gazebo's world file. This is primarily used to create a circular road of a certain radius and smoothness. The radius of the circle is calculated from the centre of the circle to the middle of the road (using Gazebo's SDF tag).

### Usage
1. Download the circle_road_gen.py script

2. Run the script
   - Open your terminal
   - Type `Python circle_road_gen.py`
   - Input your desired radius in metres
   - Input your desired smoothness in radians (lower value is smoother)
   - Copy and paste result into your world file
   
## Launch Example
1. Go to your catkin workspace
   - Launch your terminal
   - Type `cd catkin_ws`
   
2. Launch 
   - Type `roslaunch ngeeann_av_description road.launch`

## Launch Files
### gazebo.launch
Used for debugging. Launch file launches Gazebo with an empty world and spawns the ngeeann_av at the centre.

### road.launch
A foundational launch file for future launch files. Launches the road.world file into Gazebo and spawns the ngeeann_av onto the road.

### display.launch
A foundational launch file for future launch files. Launches RViz with the ngeeann_av at the centre.
