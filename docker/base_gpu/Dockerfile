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
## Run with:
##   docker run --network host --gpus all --name base_gpu --rm -it rasberry_perception:base_gpu /bin/bash
##

FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04

## Meta information
MAINTAINER Raymond Kirk    "rkirk@lincoln.ac.uk"
MAINTAINER Nikolaus Wagner "nwagner@lincoln.ac.uk"

## Install ROS
ENV ROS_DISTRO melodic
ENV TZ Europe/London
RUN rm /etc/apt/sources.list.d/* && \
    apt update && apt install -y software-properties-common lsb-release curl apt-transport-https git && \
    rm -rf /var/lib/apt/lists/* && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    apt update && \
    apt install -y "ros-$ROS_DISTRO-ros-base" python-catkin-tools python3-venv python3-pip python-rosdep && \
    rosdep init && rosdep update

## Clone necessary repositories
## TODO (melanoneiro): replace numpy with binary
WORKDIR /catkin_ws/src
RUN git clone https://github.com/RaymondKirk/rasberry_perception.git && \
    git clone https://github.com/RaymondKirk/bayestracking.git && \
    git clone https://github.com/eric-wieser/ros_numpy.git

## Install ros-dependencies and build the rasberry_perception package
WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y && \
    catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build rasberry_perception

## Create entry point for image
WORKDIR /
COPY entrypoint.sh .
ENTRYPOINT ["/bin/bash", "entrypoint.sh"]
CMD ["/bin/bash", "-c"]