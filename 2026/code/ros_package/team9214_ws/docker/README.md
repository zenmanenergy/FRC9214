# Docker images

## Ubuntu and ROS Distributions

### Ubuntu LTS Distobutions

- Ubuntu 18.04 LTS: Bionic Beaver
- Ubuntu 20.04 LTS: Focal Fossa
- Ubuntu 22.04 LTS: Jammy Jellyfish

### ROS2 LTS Distrobutions

- Jazzy Jalisco
    - Ubuntu 24.04 target
- Humble Hawksbill
    - Ubuntu 22.04 target
- Foxy Fitzroy
    - Ubuntu 20.04 target
- Dashing Diademata
    - Ubuntu 18.04 target

## Docker dev image x86_64

### Building

#### ROS Image Base with ROS2 Jazzy

`docker build -f team9214_ws/docker/Dockerfile.noble.jazzy -t team9214-ros:jazzy .`

#### Ubuntu 24.04 (Noble) with ROS2 Jazzy

`docker build -f team9214_ws/docker/ubuntu_based/Dockerfile.noble.jazzy -t team9214-noble:ros-jazzy .`

#### Ubuntu 22.04 (Jammy) with ROS2 Humble

`docker build -f team9214_ws/docker/ubuntu_based/Dockerfile.jammy.humble -t team9214-jammy:ros-humble .`

#### Ubuntu 20.04 (Focal) with ROS2 Foxy

`docker build -f team9214_ws/docker/ubuntu_based/Dockerfile.focal.foxy -t team9214-focal:ros-foxy .`

#### Ubuntu 18.04 with ROS2 dashing

`docker build -f team9214_ws/docker/ubuntu_based/Dockerfile.bionic.dashing -t team9214-bionic:ros-dashing .`


### Running

`docker run --rm -it --net=host team9214-ros:jazzy bash`