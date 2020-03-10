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
##   docker run --network host --gpus all --name detectron2 --rm -it rasberry_perception:detectron2
##

FROM rasberry_perception:base_gpu

## Meta information
MAINTAINER Raymond Kirk    "rkirk@lincoln.ac.uk"
MAINTAINER Nikolaus Wagner "nwagner@lincoln.ac.uk"

## Install backend in virtual Python3 environment
WORKDIR /
RUN python3 -m venv backend_venv --clear && \
    . backend_venv/bin/activate && \
    pip install numpy rospkg torch torchvision cython opencv-python && \
    pip install pycocotools && \
    git clone https://github.com/facebookresearch/detectron2.git && \
    cd detectron2 && pip install -e . && \
    deactivate 

## Create entry point for image
WORKDIR /
COPY entrypoint.sh .
ENTRYPOINT ["/bin/bash", "entrypoint.sh"]
CMD ["/bin/bash", "-c"]