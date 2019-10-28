FROM ros:kinetic-perception

# Meta information
MAINTAINER Raymond Kirk "rkirk@lincoln.ac.uk"

# Add dependancies
RUN apt update && apt install -y software-properties-common apt-transport-https curl python-catkin-tools git

# Add L-CAS repo
RUN curl -s http://lcas.lincoln.ac.uk/repos/public.key | apt-key add - && apt-add-repository http://lcas.lincoln.ac.uk/ubuntu/main

# Copy current rasberry_perception repo
WORKDIR /catkin_ws
COPY . /catkin_ws/src/rasberry_perception

# Install ros-dependancies
RUN rosdep install --from-paths /catkin_ws/src --ignore-src -r -y

# Build the rasberry_perception package
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin build rasberry_perception