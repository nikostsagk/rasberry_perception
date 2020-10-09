## Basis for running detection backends in a container.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##
## Build with:
##   docker build -t rasberry_perception:base_gpu .
##
## Save with:
##   docker save rasberry_perception:base_gpu | gzip > rasberry_perception_base_gpu.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name base_gpu --rm -it rasberry_perception:base_gpu /bin/bash
##

FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04

## Meta information
MAINTAINER Raymond Kirk    "rkirk@lincoln.ac.uk"

## Install ROS
ENV ROS_DISTRO melodic
ENV TZ Europe/London
RUN rm /etc/apt/sources.list.d/* && \
    DEBIAN_FRONTEND=noninteractive apt update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt install -y software-properties-common lsb-release curl apt-transport-https git --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    DEBIAN_FRONTEND=noninteractive apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-${ROS_DISTRO}-catkin --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt install -y ros-${ROS_DISTRO}-ros-base python-catkin-tools python3-venv python3-pip --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt install -y python-rosdep --no-install-recommends && rosdep init && rosdep update

## Clone necessary repositories
WORKDIR /catkin_ws/src
RUN git clone https://github.com/RaymondKirk/rasberry_perception.git && \
    git clone https://github.com/RaymondKirk/bayestracking.git

## Install ros-dependencies and build the rasberry_perception package
WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y && \
    catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build rasberry_perception

# Docker clean-up
RUN rm -rf /var/lib/apt/lists/*

## Create entry point for image
WORKDIR /
COPY entrypoint.sh .
ENTRYPOINT ["/bin/bash", "entrypoint.sh"]
CMD ["/bin/bash", "-c"]