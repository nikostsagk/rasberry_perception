## Build with:
##   docker build -t rasberry_perception:tensorrtdeepsort .
##
## Save with:
##   docker save rasberry_perception:tensorrtdeepsort | gzip >  rasberry_perception-tensorrtdeepsort.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name tensorrtdeepsort --rm -it rasberry_perception:tensorrtdeepsort
##

ARG TRT_IMAGE_VERSION=20.09
FROM nvcr.io/nvidia/tensorrt:${TRT_IMAGE_VERSION}-py3

ARG SCRIPT_DIR=/opt/tensorrt/python

RUN DEBIAN_FRONTEND=noninteractive apt-get update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ninja-build protobuf-compiler libprotobuf-dev python  python3-venv

# Copy ModularMOT as is private. Can be accessed here https://github.com/SAGARobotics/PerceptionApps
COPY ModularMOT /ModularMOT
#TEMP Copy Weights. Acquire from rbelshaw@sagarobotics.com 
COPY mars_sb_14.pb /mars_sb_14.pb
COPY yolov4-416.onnx /ModularMOT/modularmot/models/yolov4-416.onnx

WORKDIR /
RUN git clone https://github.com/rjwb1/deep_sort.git

## Install backend in virtual Python3 environment
RUN python3.6 -m venv modularmot_venv --clear --system-site-packages && \
    . modularmot_venv/bin/activate && \
    pip install --no-cache-dir rospkg && \
    dpkg -i ${SCRIPT_DIR}/*-tf_*.deb && \
    pip install --no-cache-dir numpy && \
    pip install --no-cache-dir cython && \
    pip install --no-cache-dir cython-bbox && \
    pip install --no-cache-dir -r /ModularMOT/requirements.txt && \
    pip install --no-cache-dir scikit-build && \
    pip install --no-cache-dir opencv-python && \
    pip install --no-cache-dir scipy tensorflow && \
    pip install --no-cache-dir scikit-learn==0.22.2

ENV PYTHONPATH "${PYTHONPATH}:/ModularMOT:/deep_sort"

## Install ROS
ENV ROS_DISTRO melodic
ENV TZ Europe/London
RUN DEBIAN_FRONTEND=noninteractive apt update --no-install-recommends && \
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
RUN git clone https://github.com/SagaRobotics/rasberry_perception.git
WORKDIR /ModularMOT/modularmot/plugins
RUN make

## Install ros-dependencies and build the rasberry_perception package
WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y && \
    catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build rasberry_perception

WORKDIR /

# Docker clean-up
RUN rm -rf /var/lib/apt/lists/*

## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]