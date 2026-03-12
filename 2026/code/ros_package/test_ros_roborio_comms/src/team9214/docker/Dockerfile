FROM nvcr.io/nvidia/l4t-base:r32.7.1 AS builder

WORKDIR /tmp

COPY docker/install_artifacts/* /tmp/

# # Install base dependencies
# RUN apt-get update && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends $(cat /tmp/apt_base_packages.txt) && \
#     # Clean up the apt lists and the temporary file to reduce image size
#     rm -rf /var/lib/apt/lists/* && \
#     rm /tmp/apt_base_packages.txt

# Install build dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    gnupg2 \
    cmake \
    curl \
    git \
    lsb-release \
    && rm -rf /var/lib/apt/lists/* 

# # Install ROS 2 Humble build tools
# RUN apt-get update && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends $(cat /tmp/apt_ros_packages.txt) && \
#     # Clean up the apt lists and the temporary file to reduce image size
#     rm -rf /var/lib/apt/lists/* && \
#     rm /tmp/apt_ros_packages.txt

# Install ROS 2 Humble build tools
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros-latest.list \
    && apt-get update && apt-get install -y \
    python3-dev \
    python3-pip \
    python3-catkin-tools \
    python3-catkin-pkg \
    python3-catkin-lint \
    ros-Humble-base \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-* \
    && rm -rf /var/lib/apt/lists/*

# Install pyntcore
# RUN python3 -m pip install --upgrade pip \
#         setuptools \
#         wheel \
#         pyyaml \
#         jinja2 \
#         typeguard \
#         lark \
#         pyntcore \
#         robotpy

RUN pip install -r /tmp/package_lists/requirements.txt

WORKDIR /workspace
COPY ../../../src/* /workspace/src

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Set up entry point to source workspace and run both services
RUN echo '#!/bin/bash\n\
. /opt/ros/humble/setup.sh\n\
. /workspace/install/setup.sh\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh