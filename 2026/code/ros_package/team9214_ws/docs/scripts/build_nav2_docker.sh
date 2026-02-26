#!/bin/bash

export ROS_DISTRO=rolling
git clone https://github.com/ros-navigation/navigation2.git --branch main
docker build --tag navigation2:$ROS_DISTRO \
  --build-arg FROM_IMAGE=ros:$ROS_DISTRO \
  --build-arg OVERLAY_MIXINS="release ccache lld" \
  --cache-from ghcr.io/ros-navigation/navigation2:main \
  ./navigation2
