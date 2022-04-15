#!/bin/bash

# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

# gazebo install
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7 gazebo7-common -y
sudo apt-get install libgazebo7-dev -y

# other apt-get installs
sudo apt-get install git -y
sudo apt-get install joint-state-publisher -y
sudo apt-get install ros-kinetic-joint-state-publisher-gui -y
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers -y
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control -y
sudo apt-get install python-rospy -y
sudo apt-get install python-pip -y

# pip installs
pip install --upgrade pip
pip install -r requirements.txt
