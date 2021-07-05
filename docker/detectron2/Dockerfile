## Basis for running detectron2 backend in a container.
## Extended from the saga/rasberry_perception:base_gpu image.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##
## Build with:
##   docker build -t saga/rasberry_perception:detectron2 .
##
## Save with:
##   docker save saga/rasberry_perception:detectron2 | gzip > saga_rasberry_perception_detectron2.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name detectron2_backend --rm -it saga/rasberry_perception:detectron2
## Debug with:
##   docker run --network host --gpus all --name detectron2_backend --entrypoint /bin/bash --rm -it saga/rasberry_perception:detectron2

FROM saga/rasberry_perception:base_gpu

## Meta information
LABEL detectron2.version="0.4" maintainers="Saul Goldblatt <saul.goldblatt@sagarobotics.com>, Nikos Tsagkopoulos <ntsagkopoulos@sagarobotics.com"

# Set FORCE_CUDA because during `docker build` cuda is not accessible
ENV FORCE_CUDA="1"
# This will by default build detectron2 for all common cuda architectures and take a lot more time,
# because inside `docker build`, there is no way to tell which architecture will be used.
ARG TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
ENV TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}"
# Set a fixed model cache directory if you ever want to mount volumes for models.
ENV FVCORE_CACHE="/tmp"

WORKDIR /

RUN DEBIAN_FRONTEND=noninteractive apt-get update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ninja-build protobuf-compiler libprotobuf-dev --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

## Install backend in virtual Python3 environment
## Install dependencies. See https://pytorch.org/ for other options if you use a different version of CUDA
RUN python3 -m venv detectron2_venv --clear && \
    . detectron2_venv/bin/activate && \
    pip install --upgrade pip wheel setuptools && \
    pip install --no-cache-dir tensorboard cython && \
    pip install --no-cache-dir opencv-python numpy && \
    pip install --no-cache-dir torch torchvision torchaudio -f https://download.pytorch.org/whl/lts/1.8/torch_lts.html && \
    pip install --no-cache-dir 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI' && \
    pip install --no-cache-dir 'git+https://github.com/facebookresearch/fvcore' && \
    # Install detectron2
    python -m pip install detectron2==0.4 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu102/torch1.8/index.html && \
    pip install --no-cache-dir rospkg

## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]