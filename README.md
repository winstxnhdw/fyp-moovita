# AUTONOMOUS VEHICLE: CONTROL AND BEHAVIOUR
### Ngee Ann Polytechnic Engineering Science Final Year Project with MooVita, 2020
![ngeeann_av](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/pictures/ngeeann_av_ultrawide.png?raw=true)

## Abstract
This project covers the development of a robust non-holonomic autonomous vehicle platform in a simulated environment using ROS and Gazebo. A sense-think-act cycle is implemented to navigate the virtual world, avoiding static and moving objects.
<br />
<br />
![Obstacle Avoidance](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/obstacle_avoidance.gif?raw=true)

## Table of Contents
- [Abstract](#Abstract)
- [Requirements](#Requirements)
  - [Operating System](#Operating-System)
  - [Software](#Software)
- [Installation](#Installation)
- [Quick Start](#Quick-Start)
- [Scripts](#Scripts)
  - [circle_road_gen.py](#circle_road_gen.py)
    - [Description](#Description)
    - [Usage](#Usage)
  - [circle_wp_gen.py](#circle_wp_gen.py)
    - [Description](#Description)
    - [Point mode](#Point-mode)
    - [Angle mode](#Angle-mode)
    - [Usage](#Usage)
- [Launch Files](#Launch-Files)
  - [ackermann_vehicle.launch](#ackermann_vehicle.launch)
  - [ackermann_controller.launch](#ackermann_controller.launch)
- [Renders](#Renders)

## Requirements
### Hardware
Recommended its equivalent or higher:
- Intel Core i7-8700 Desktop Processor
- NVIDIA GeForce GTX 1080

### Operating System
1. [Ubuntu 16.04.6 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/)

### Software
1. [Desktop-Full ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
   - [joint_state_publisher](http://wiki.ros.org/joint_state_publisher)
   - joint_state_publisher_gui
   - [ros_controller](http://wiki.ros.org/ros_control#Install)
	 
2. [Python 2.7](https://www.python.org/download/releases/2.7/)
   - [pip](https://pypi.org/project/pip/)
   - [rospy](http://wiki.ros.org/rospy)
   - [NumPy](https://pypi.org/project/numpy/)
   - [pandas](https://pandas.pydata.org/getting_started.html)
  
3. [Gazebo 7.16](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install)
   - [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

4. [Git](https://git-scm.com/download/linux)

5. [ackermann_msgs](https://github.com/ros-drivers/ackermann_msgs.git)

## Installation
1. Install [Ubuntu 16.04.6 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/)

2. Git clone this repository
   - Open your terminal
   - Go to the directory you wish to clone the repository in
   - Type `git clone https://github.com/winstxnhdw/AutoCarROS.git`
   
3. Change directory to your cloned path
   - Go to your terminal
   - Type `cd <workspace>/src/fyp-moovita`
   
4. Install [Desktop-Full ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
   - Type `chmod +x ros-kinetic-desktop-full-install.sh`
   - Type `./ros-kinetic-desktop-full-install.sh` to install Desktop-Full ROS Kinetic
  
5. Install the required packages
   - Type `chmod +x requirements.sh`
   - Type `./requirements.sh` 
   
## Quick Start
1. Launch ngeeann_av.launch
   - Launch your terminal
   - Type `catkin_make`
   - Type `roslaunch launches ngeeann_av.launch`
2. Execute tracker.py
   - Type `rosrun ngeeann_av_nav tracker.py`

## Scripts
### circle_road_gen.py
![Road Generation](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/road_gen.gif?raw=true)

#### Description
circle_road_gen.py is a custom script which uses the NumPy library to calculate and generate the three-dimensional <point> coordinates of a circle for Gazebo's world file. This is primarily used to create a circular road of a certain radius and smoothness. The radius of the circle is calculated from the centre of the circle to the middle of the road (using Gazebo's SDF tag).

#### Usage
1. Download the circle_road_gen.py script if you have not cloned this repository

2. Go to the script's directory
   - Open your terminal
   - Type `cd scripts`

3. Run the script
   - Type `python circle_road_gen.py`
   - Input your desired radius in metres
   - Input your desired smoothness in degrees (lower value is smoother)
   - Copy and paste result into your world file
  
### circle_wp_gen.py
![circle_wp_gen](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/pictures/circle_wp_gen.png?raw=true)
#### Description
circle_wp_gen.py is a custom script which uses the NumPy and pandas library to calculate and generate a csv file that contains the two-dimensional coordinates; x-axis and y-axis in their respective columns. This is primarily used to generate waypoints on a circular road of a certain radius and smoothness. The user is given two different smoothness modes; Point mode and Angle mode. The radius of the circle is calculated from the centre of gazebo world.

#### Point mode
The user is able to choose how many waypoints to generate, and the smoothness of the circular waypoint is based on how many points the user has set. More points means a smoother waypoint

#### Angle mode
The user is unable to choose how many waypoints to generate, and the smoothness of the circular waypoint is based on the degree value the user has set. Lower value means a smoother waypoint.

#### Usage
1. Download the circle_wp_gen.py script if you have not cloned this repository.

2. Go to the script's directory
   - Open your terminal
   - Type `cd scripts`
   
3. Run the script
   - Type `python circle_wp_gen.py`
   - Choose your desired smoothness mode
   - Input your desired radius in metres
   - Input your desired smoothness in number of points (if you chose Point mode) or degrees (if you chose Angle mode)

## Launch Files
|Launch File|Launches|Purpose|
|-----------|--------|-------|
|gazebo.launch|gazebo, no world, ngeeann_av|For debugging
|road.launch|gazebo, road.world, ngeeann_av|Foundational launch file for future launch files
|display.launch|rviz, ngeeann_av|Foundational launch file for future launch files
|controller.launch|axle controllers, steer controllers|Foundational launch file for future launch files

### ackermann_vehicle.launch
Launches the populated_road.world file into Gazebo and spawns the ngeeann_av onto a populated road world. It also launches ackermann_controller.launch, RViz, the controller spawner and ackermann controller. If your Gazebo does not start, this is because you do not have the required Gazebo models in your models folder. To fix this, you may change the ackermann_vehicle.launch parameters to launch the unpopulated road variant, road.launch.

### ackerman_controller.launch
Launches nodes used by both RViz and Gazebo when visualizing a vehicle with Ackermann steering.
## Renders
"<b>Because work is meaningless if it doesn't look cool."<b> 
	
<br />
![Renders](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/renders.gif?raw=true)
![1](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/1.gif?raw=true)
![2](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/2.gif?raw=true)
![3](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/3.gif?raw=true)
![4](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/4.gif?raw=true)
