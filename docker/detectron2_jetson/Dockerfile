## Basis for running detectron2 backend in a container on a JETSON.
## Extended from the nvcr.io/nvidia/l4t-pytorch:r32.5.0-pth1.7-py3 image.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##
## Build with:
##   docker build -t saga/rasberry_perception:detectron2_jetson .
##
## Save with:
##   docker save saga/rasberry_perception:detectron2_jetson | gzip > saga_rasberry_perception_detectron2_jetson.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name detectron2_backend --rm -it saga/rasberry_perception:detectron2_jetson
## Debug with:
##   docker run --network host --gpus all --name detectron2_backend --entrypoint /bin/bash --rm -it saga/rasberry_perception:detectron2_jetson

FROM nvcr.io/nvidia/l4t-pytorch:r32.5.0-pth1.7-py3

## Meta information
LABEL detectron2.version="0.4" cuda.version="10.2" maintainers="Saul Goldblatt <saul.goldblatt@sagarobotics.com>, Nikos Tsagkopoulos <ntsagkopoulos@sagarobotics.com"

# Set FORCE_CUDA because during `docker build` cuda is not accessible
ENV FORCE_CUDA="1"
# This will by default build detectron2 for all common cuda architectures and take a lot more time,
# because inside `docker build`, there is no way to tell which architecture will be used.
ARG TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
ENV TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}"
# Set a fixed model cache directory if you ever want to mount volumes for models.
ENV FVCORE_CACHE="/tmp"

WORKDIR /

## Install ROS
ENV ROS_DISTRO melodic
ENV TZ Europe/London
RUN rm /etc/apt/sources.list.d/* && \
    DEBIAN_FRONTEND=noninteractive apt-get update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y software-properties-common lsb-release curl apt-transport-https git --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ninja-build protobuf-compiler libprotobuf-dev --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    ## https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    DEBIAN_FRONTEND=noninteractive apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-${ROS_DISTRO}-catkin --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-${ROS_DISTRO}-ros-base  --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y python-catkin-tools python3-venv python3-pip python-rosdep --no-install-recommends && rosdep init && rosdep update

## Install backend in virtual Python3 environment
RUN python3 -m venv detectron2_venv --clear && \
    . detectron2_venv/bin/activate && \
    pip install --upgrade pip wheel setuptools && \
    pip install --no-cache-dir tensorboard cython && \
    pip install --no-cache-dir opencv-python numpy && \
    pip install --no-cache-dir 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI' && \
    pip install --no-cache-dir 'git+https://github.com/facebookresearch/fvcore' && \
    # Install detectron2
    python -m pip install detectron2==0.4 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu102/torch1.7/index.html && \
    pip install --no-cache-dir rospkg

## Clone necessary repositories
#TODO: replace with sagarobotics forks
WORKDIR /catkin_ws/src
RUN git clone https://github.com/saulgoldsaga/rasberry_perception.git

## Install ros-dependencies and build the rasberry_perception package
WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build rasberry_perception

## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]