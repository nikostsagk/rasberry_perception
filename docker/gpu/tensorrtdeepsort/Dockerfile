## Build with:
##   docker build -t repository.sagarobotics.com/tensorrt_yolov4deepsort.gpu:local .
##
## Save with:
##   docker save repository.sagarobotics.com/tensorrt_yolov4deepsort.gpu:local | gzip > rasberry_perception_tensorrt_yolov4deepsort_gpu.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name tensorrt_yolov4deepsort_backend --rm -it repository.sagarobotics.com/tensorrt_yolov4deepsort.gpu:local
## Debug with:
##   docker run --network host --gpus all --name tensorrt_yolov4deepsort_backend --entrypoint /bin/bash --rm -it repository.sagarobotics.com/tensorrt_yolov4deepsort.gpu:local
## Tag with:
##   docker tag repository.sagarobotics.com/tensorrt_yolov4deepsort.gpu:local repository.sagarobotics.com/tensorrt_yolov4deepsort.gpu:$(git show --format="%h" -s)

FROM nvcr.io/nvidia/tensorrt:20.09-py3

LABEL cuda.version="11.0" cudnn.version="8.0" tensorrt.version="7.1.3" maintainers="Robert Belshaw <rbelshaw@sagarobotics.com>, Nikos Tsagkopoulos <ntsagkopoulos@sagarobotics.com>"

RUN DEBIAN_FRONTEND=noninteractive apt-get update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ninja-build protobuf-compiler libprotobuf-dev python  python3-venv

#TEMP Copy Weights. Acquire from rbelshaw@sagarobotics.com 
COPY mars_sb_14.pb /mars_sb_14.pb
COPY yolov4-416.onnx /PerceptionApps/ModularMOT/modularmot/models/yolov4-416.onnx
COPY tensorrtdeepsort.json /config.json

WORKDIR /
RUN git clone https://github.com/rjwb1/deep_sort.git
COPY PerceptionApps /PerceptionApps
RUN cd /PerceptionApps && git checkout ModularMOT && \
    cd /PerceptionApps/ModularMOT/modularmot/plugins && \
    make

## Install backend in virtual Python3 environment
WORKDIR /
RUN python3.6 -m venv modularmot_venv --clear --system-site-packages && \
    . modularmot_venv/bin/activate && \
    pip install --no-cache-dir rospkg && \
    dpkg -i /opt/tensorrt/python/*-tf_*.deb && \
    pip install --no-cache-dir numpy && \
    pip install --no-cache-dir cython && \
    pip install --no-cache-dir cython-bbox && \
    pip install --no-cache-dir -r /PerceptionApps/ModularMOT/requirements.txt && \
    pip install --no-cache-dir scikit-build && \
    pip install --no-cache-dir opencv-python && \
    pip install --no-cache-dir scipy tensorflow && \
    pip install --no-cache-dir scikit-learn==0.22.2

ENV PYTHONPATH "${PYTHONPATH}:/PerceptionApps/ModularMOT:/deep_sort"

## Install ROS
ENV ROS_DISTRO melodic
ENV TZ Europe/London
RUN DEBIAN_FRONTEND=noninteractive apt update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt install -y software-properties-common lsb-release curl apt-transport-https git --no-install-recommends && \
    rm -rf /var/lib/apt/lists/* && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    DEBIAN_FRONTEND=noninteractive apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-${ROS_DISTRO}-catkin --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt install -y ros-${ROS_DISTRO}-ros-base python-catkin-tools python3-venv python3-pip --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt install -y python-rosdep --no-install-recommends && rosdep init && rosdep update

## Clone necessary repositories
WORKDIR /catkin_ws/src
RUN git clone https://github.com/SagaRobotics/rasberry_perception.git

## Install ros-dependencies and build the rasberry_perception package
WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y && \
    catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build rasberry_perception

# set up environment variables
ENV PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

WORKDIR /
RUN . modularmot_venv/bin/activate && /PerceptionApps/ModularMOT/modularmot/build_engine.py "/config.json"

# Docker clean-up
RUN rm -rf /var/lib/apt/lists/*

## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
WORKDIR /
COPY entrypoint.sh .
COPY start_backend.sh .
ENTRYPOINT ["/bin/bash", "entrypoint.sh"]
CMD ["/bin/bash", "-c"]
