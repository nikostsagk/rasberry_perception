## Basis for running fruitcast backend in a container.
## Extended from the rasberry_perception:base_gpu image.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##
## Build with:
##   docker build -t rasberry_perception:fruitcast .
##
## Save with:
##   docker save rasberry_perception:fruitcast | gzip > rasberry_perception_fruitcast.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name fruitcast_backend --rm -it rasberry_perception:fruitcast
##

FROM rasberry_perception:base_gpu

## Meta information
MAINTAINER Raymond Kirk    "rkirk@lincoln.ac.uk"

# Set FORCE_CUDA because during `docker build` cuda is not accessible
ENV FORCE_CUDA="1"
# This will by default build fruitcast for all common cuda architectures and take a lot more time,
# because inside `docker build`, there is no way to tell which architecture will be used.
ARG TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
ENV TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}"

WORKDIR /

## Install fruitcast:yolov5 from https://github.com/raymondkirk/yolov5
COPY yolov5 /yolov5/

## Add backends. These files can be obtained from Raymond (rkirk@lincoln.ac.uk) and require citation when used.
COPY best.pt /best.pt

RUN DEBIAN_FRONTEND=noninteractive apt-add-repository -y ppa:deadsnakes/ppa && \
    DEBIAN_FRONTEND=noninteractive apt-get update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ninja-build protobuf-compiler libprotobuf-dev python3.8* --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

## Install backend in virtual Python3 environment
RUN python3.8 -m venv fruitcast_venv --clear && \
    . fruitcast_venv/bin/activate && \
    # Install dependencies. See https://pytorch.org/ for other options if you use a different version of CUDA
    pip install --no-cache-dir -r yolov5/requirements.txt && \
    pip install --no-cache-dir -e yolov5 && \
    pip install --no-cache-dir rospkg


## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]