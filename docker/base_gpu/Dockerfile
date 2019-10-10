FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu16.04

# Meta information
MAINTAINER Raymond Kirk "rkirk@lincoln.ac.uk"

# Install ROS + LCAS
RUN rm /etc/apt/sources.list.d/* && \
    apt update && apt install -y software-properties-common lsb-release curl apt-transport-https git && \
    rm -rf /var/lib/apt/lists/* && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt update && \
    apt install -y ros-kinetic-ros-base python-catkin-tools && \
    rosdep init && \
    rosdep update

# Copy current rasberry_perception repo
WORKDIR /catkin_ws
COPY . /catkin_ws/src/rasberry_perception
RUN git clone https://github.com/RaymondKirk/pico_zense_camera src/pico_zense_camera

# Install ros-dependancies and Build the rasberry_perception package
ENV ROS_DISTRO kinetic
RUN rosdep install --from-paths /catkin_ws/src --ignore-src -r -y && \
    catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build rasberry_perception

# Create entry point for image
COPY docker/base_gpu/entry_point.sh .
ENTRYPOINT ["/bin/bash", "entry_point.sh"]
CMD ["/bin/bash", "-c"]