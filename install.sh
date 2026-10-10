#!/bin/bash
sudo apt update

sudo apt install -y python3-colcon-common-extensions python3-vcstool git
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev
sudo apt install -y ros-humble-gazebo-ros2-control ros-humble-ros2-controllers
sudo apt install -y ros-humble-nav2-bringup ros-humble-nav2-map-server
sudo apt install -y ros-humble-robot-localization ros-humble-slam-toolbox
sudo apt install -y ros-humble-velodyne-simulator ros-humble-spatio-temporal-voxel-layer
sudo apt install -y ros-humble-xacro ros-humble-joint-state-publisher
ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-ecl-angles ros-humble-ecl-threads \
  libboost-program-options-dev nlohmann-json3-dev
sudo apt install -y python3-transforms3d python3-serial
sudo apt install -y ros-humble-mavros-msgs
