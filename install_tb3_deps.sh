#!/bin/bash


set -e

sudo apt update -y

echo "ROS 2 Package installation"
# On remplace 'jazzy' par 'humble' partout
# On ajoute 'python3-lxml' pour corriger le bug de spawn
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-turtlebot3-gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-robot-state-publisher \
  python3-pip \
  python3-lxml

pip install --upgrade pip
pip install matplotlib

