cmake_minimum_required(VERSION 3.0.2)
project(ngeeann_av_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
  FILES
  State2D.msg
  Twist2D.msg
  Path2D.msg
  AckermannDrive.msg
  AckermannDriveStamped.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  std_msgs
  DEPENDS 
  Boost
)
