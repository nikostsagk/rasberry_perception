## Basis for running detectron2 backend in a container on a JETSON.
## Extended from the saga/rasberry_perception:base_jetson image.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##   - set nvidia as the default runtime configuration
##
## Important information:
##   https://stackoverflow.com/a/61737404/11750663
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

FROM saga/rasberry_perception:base_jetson

## Meta information
LABEL detectron2.version="0.4" maintainers="Saul Goldblatt <saul.goldblatt@sagarobotics.com>, Nikos Tsagkopoulos <ntsagkopoulos@sagarobotics.com"

# # Set FORCE_CUDA because during `docker build` cuda is not accessible
ENV FORCE_CUDA="1"
# # This will by default build detectron2 for all common cuda architectures and take a lot more time,
# # because inside `docker build`, there is no way to tell which architecture will be used.
ARG TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
ENV TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}"
# # Set a fixed model cache directory if you ever want to mount volumes for models.
ENV FVCORE_CACHE="/tmp"

WORKDIR /

RUN DEBIAN_FRONTEND=noninteractive apt-get update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ninja-build protobuf-compiler libprotobuf-dev python3-venv --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

## Install backend in virtual Python3 environment and dependencies
RUN python3 -m venv detectron2_venv --system-site-packages && \
    . /detectron2_venv/bin/activate && \
    python -m pip install --upgrade pip wheel setuptools && \
    python -m pip install --no-cache-dir tensorboard Cython && \
    python -m pip install --no-cache-dir opencv-python numpy && \
    # Install detectron2
    python -m pip install 'git+git://github.com/facebookresearch/detectron2.git@v0.4' && \
    python -m pip install --no-cache-dir rospkg

## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]