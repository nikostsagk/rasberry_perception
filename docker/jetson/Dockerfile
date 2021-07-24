## Build with:
##   docker build -t repository.sagarobotics.com/base.jetson:local .
##
## Save with:
##   docker save repository.sagarobotics.com/base.jetson:local | gzip > base_jetson.tar.gz
##
## Debug with:
##   docker run --network host --gpus all --name base_backend --entrypoint /bin/bash --rm -it repository.sagarobotics.com/base.jetson:local
## Tag with:
##   docker tag repository.sagarobotics.com/base.jetson:local repository.sagarobotics.com/base.jetson:$(git show --format="%h" -s)


FROM nvcr.io/nvidia/l4t-pytorch:r32.5.0-pth1.7-py3

## Meta information
LABEL cuda.version="10.2" pytorch.version="1.7" maintainers="Saul Goldblatt <saul.goldblatt@sagarobotics.com>, Nikos Tsagkopoulos <ntsagkopoulos@sagarobotics.com>"

## Install ROS
ENV ROS_DISTRO melodic
ENV TZ Europe/London
RUN DEBIAN_FRONTEND=noninteractive apt-get update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y software-properties-common lsb-release curl apt-transport-https git gpg-agent --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    ## https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    DEBIAN_FRONTEND=noninteractive apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-${ROS_DISTRO}-catkin --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-${ROS_DISTRO}-ros-base  --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y python-catkin-tools python3-pip python-rosdep --no-install-recommends && rosdep init && rosdep update

## Clone necessary repositories
#TODO: replace with sagarobotics forks
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