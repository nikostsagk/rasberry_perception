## Basis for running detectron2 backend in a container.
## Extended from the rasberry_perception:base_gpu image.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##
## Build with:
##   docker build -t rasberry_perception:detectron2 .
##
## Run with:
##   docker run --network host --gpus all --name base_gpu --rm -it rasberry_perception:detectron2
##

FROM rasberry_perception:base_gpu

## Meta information
MAINTAINER Raymond Kirk    "rkirk@lincoln.ac.uk"
MAINTAINER Nikolaus Wagner "nwagner@lincoln.ac.uk"

RUN DEBIAN_FRONTEND=noninteractive apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
	python3-opencv ca-certificates python3-dev git wget sudo  \
	cmake ninja-build protobuf-compiler libprotobuf-dev && \
    rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USER_ID=1000
RUN useradd -m --no-log-init --system  --uid ${USER_ID} appuser -g sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER appuser
WORKDIR /home/appuser

## Install backend in virtual Python3 environment
RUN python3 -m venv detectron2_venv --clear && \
    . detectron2_venv/bin/activate

ENV PATH="/home/appuser/.local/bin:${PATH}"
RUN wget https://bootstrap.pypa.io/get-pip.py && \
	python3 get-pip.py --user && \
	rm get-pip.py

# set FORCE_CUDA because during `docker build` cuda is not accessible
ENV FORCE_CUDA="1"

# install dependencies
# See https://pytorch.org/ for other options if you use a different version of CUDA
RUN . detectron2_venv/bin/activate && \
    pip install tensorboard cython && \
    pip install opencv-python numpy && \
    # At time of writing CUDA 10.2 is the native torch version
    #    pip install torch==1.5+cu102 torchvision==0.6+cu102 -f https://download.pytorch.org/whl/torch_stable.html && \
    pip install torch torchvision && \
    pip install 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI' && \
    pip install 'git+https://github.com/facebookresearch/fvcore'

# install detectron2
RUN git clone https://github.com/facebookresearch/detectron2

# This will by default build detectron2 for all common cuda architectures and take a lot more time,
# because inside `docker build`, there is no way to tell which architecture will be used.
ARG TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
ENV TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}"

# Install required ros packages and detectron2 locally
RUN . detectron2_venv/bin/activate && \
    pip install -e detectron2 && \
    pip install rospkg

## Add backends
## TODO: These files can be obtained from Raymond and require citation
COPY r50_packaged /r50_packaged/
# Set a fixed model cache directory.
ENV FVCORE_CACHE="/tmp"

## Create entry point for image
WORKDIR /
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]