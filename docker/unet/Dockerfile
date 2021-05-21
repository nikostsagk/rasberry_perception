## Basis for running unet backend in a container.
## Extended from the rasberry_perception:saga_base_gpu image.
##
## Prerequisites:
##   - docker
##   - nvidia driver
##   - nvidia container toolkit
##
## Build with:
##   docker build -t rasberry_perception:unet .
##
## Save with:
##   docker save rasberry_perception:unet | gzip > rasberry_perception_unet.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name unet_backend --rm -it rasberry_perception:unet
##
FROM saulgold/saga_perception:saga_base_gpu
MAINTAINER Saul Goldblatt

# Set FORCE_CUDA because during `docker build` cuda is not accessible
ENV FORCE_CUDA="1"
# This will by default build for all common cuda architectures and take a lot more time,
# because inside `docker build`, there is no way to tell which architecture will be used.
ARG TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
ENV TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}"

WORKDIR /
COPY epoch_80.pth param.yaml /
COPY berry_segmentation /berry_segmentation

#activate conda env
RUN conda env create -f /berry_segmentation/berry_segmentation_conda_env.yml
SHELL ["conda", "run", "-n", "torch", "/bin/bash", "-c"]
# Make sure the environment is activated:
RUN echo "Make sure pytorch is installed:"
RUN python -c "import torch"
## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]
