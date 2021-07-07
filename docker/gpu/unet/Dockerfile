## Basis for running unet backend in a container.
## Extended from the rasberry_perception:gpu image.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##
## Build with:
##   docker build -t rasberry_perception/unet:gpu .
##
## Save with:
##   docker save rasberry_perception/unet:gpu | gzip > rasberry_perception_unet_gpu.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name unet_backend --rm -it rasberry_perception/unet:gpu
##
FROM rasberry_perception:gpu
LABEL maintainer="Saul Goldblatt <saul.goldblatt@sagarobotics.com>"

# Set FORCE_CUDA because during `docker build` cuda is not accessible
ENV FORCE_CUDA="1"
# This will by default build for all common cuda architectures and take a lot more time,
# because inside `docker build`, there is no way to tell which architecture will be used.
ARG TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
ENV TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}"


WORKDIR /
COPY epoch_80.pth param.yaml /
COPY berry_segmentation /catkin_ws/src/rasberry_perception/src/rasberry_perception/berry_segmentation
#
RUN DEBIAN_FRONTEND=noninteractive curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    DEBIAN_FRONTEND=noninteractive apt-get update --no-install-recommends && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -y ninja-build    protobuf-compiler libprotobuf-dev python3.8* wget --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*
#
# ## Install backend in virtual Python3 environment
RUN python3.6 -m venv unet_venv --clear && \
    . unet_venv/bin/activate && \
    pip install --upgrade pip && \
    pip install --no-cache-dir -r /catkin_ws/src/rasberry_perception/src/rasberry_perception/berry_segmentation/requirements.txt

## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]