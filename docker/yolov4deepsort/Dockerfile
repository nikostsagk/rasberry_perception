## Basis for running yolovdeepsort backend in a container.
## Extended from the rasberry_perception:base_gpu image.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##
## Build with:
##   docker build -t rasberry_perception:yolov4deepsort .
##
## Save with:
##   docker save rasberry_perception:yolov4deepsort | gzip > rasberry_perception_yolov4deepsort.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name yolov4deepsort_backend --rm -it rasberry_perception:yolov4deepsort
##

FROM rasberry_perception:base_gpu

## Meta information
MAINTAINER Robert Belshaw    "rbelshaw@sagarobotics.com"

# Set FORCE_CUDA because during `docker build` cuda is not accessible
ENV FORCE_CUDA="1"
# This will by default build fruitcast for all common cuda architectures and take a lot more time,
# because inside `docker build`, there is no way to tell which architecture will be used.
ARG TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
ENV TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}"

#TEMP 
WORKDIR /catkin_ws/src/rasberry_perception
RUN git remote add saga https://github.com/SAGARobotics/rasberry_perception.git && \
    git fetch saga && \
    git checkout tracking_backends_integration && \
    git pull saga tracking_backends_integration
#TEMP 

## Add Darknet. These files can be obtained from Rob ("rbelshaw@sagarobotics.com") or downloaded from https://github.com/SAGARobotics/Noronn.git branch cuda10
## The Makefile should be tweaked to build for the hardware that you wish to implement on.
COPY Noronn /catkin_ws/src/Noronn
WORKDIR /catkin_ws/src/Noronn/deep_sort_yolov4-master
RUN git checkout cuda10 && \
    make 
WORKDIR /

RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y python-pip 
RUN pip install --no-cache-dir scipy scikit-learn tensorflow

ENV PYTHONPATH "${PYTHONPATH}:/catkin_ws/src/Noronn/deep_sort_yolov4-master"

## Add backends. These files can be obtained from Rob ("rbelshaw@sagarobotics.com") 
COPY mars_sb_14.pb /mars_sb_14.pb
COPY yolov4_sb_best.weights /yolov4_sb_best.weights

## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]