#!/bin/bash

# Exit immediatelly if a command exits with a non-zero status
set -e

# Executes a command when DEBUG signal is emitted in this script - should be after every line
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# Executes a command when ERR signal is emmitted in this script
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd "$MY_PATH/.."

# Install Cartographer specific dependencies
sudo apt-get update
sudo apt-get install -y \
    libyaml-dev \
    libblas-dev \
    liblapack-dev \
    libompl-dev \
    ros-$ROS_DISTRO-octomap-ros \
    ros-$ROS_DISTRO-octomap \
    ros-$ROS_DISTRO-octomap-rviz-plugins \
    ros-$ROS_DISTRO-ompl \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-visual-tools \
    ros-$ROS_DISTRO-dynamixel-workbench-msgs

# Check for octomap_server
num=$(dpkg --list | grep ros-$ROS_DISTRO-octomap-server | wc -l)
if [ "$num" -gt "0" ]; then
  sudo apt-get purge -y ros-$ROS_DISTRO-octomap-server
fi

# Install Gitman
curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/dependencies/gitman.sh | bash

# Install Gitman pckages
gitman install --force
