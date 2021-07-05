## Basis for running detection backends in a container.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##
## Build with:
##   docker build -t rasberry_perception:gpu .
##
## Save with:
##   docker save rasberry_perception:gpu | gzip > rasberry_perception_gpu.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name gpu --rm -it rasberry_perception:gpu /bin/bash
##

FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04

## Meta information
LABEL cuda.version="10.2" maintainers="Saul Goldblatt <saul.goldblatt@sagarobotics.com>, Nikos Tsagkopoulos <ntsagkopoulos@sagarobotics.com>"

## Install ROS
ENV ROS_DISTRO melodic
ENV TZ Europe/London
RUN rm /etc/apt/sources.list.d/* && \
    DEBIAN_FRONTEND=noninteractive apt-get update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y software-properties-common lsb-release curl apt-transport-https git --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    ## https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    DEBIAN_FRONTEND=noninteractive apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-${ROS_DISTRO}-catkin --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-${ROS_DISTRO}-ros-base  --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y python-catkin-tools python3-venv python3-pip python-rosdep --no-install-recommends && rosdep init && rosdep update

## Clone necessary repositories
WORKDIR /catkin_ws/src
RUN git clone https://github.com/SAGARobotics/rasberry_perception.git

## Install ros-dependencies and build the rasberry_perception package
WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build rasberry_perception

# Docker clean-up
RUN rm -rf /var/lib/apt/lists/*

## Create entry point for image
WORKDIR /
COPY entrypoint.sh .
ENTRYPOINT ["/bin/bash", "entrypoint.sh"]
CMD ["/bin/bash", "-c"]