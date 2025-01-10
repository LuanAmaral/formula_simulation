#!/bin/bash

#  check if git repositores are cloned
git clone 'https://gitlab.com/eufs/eufs_sim' src/eufs_sim 
git clone 'https://gitlab.com/eufs/eufs_msgs' src/eufs_msgs
git clone 'https://gitlab.com/eufs/eufs_rviz_plugins' src/eufs_rviz_plugins

# Source ROS2 
source /opt/ros/galactic/setup.bash

# Install dependencies
apt update && apt-get update && apt-get install -y \
    ros-galactic-gazebo-ros \
    ros-galactic-rviz2 \
    ros-galactic-joint-state-publisher \
    ros-galactic-ackermann-msgs \
    ros-galactic-xacro \
    ros-galactic-yaml-cpp-vendor \
    libyaml-cpp-dev &&\


rm -rf /var/lib/apt/lists/*


# Build ROS2 workspace
cd /ros2_ws
colcon build

# Source workspace
source /ros2_ws/install/setup.bash