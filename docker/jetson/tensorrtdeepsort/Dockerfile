## Build with:
##   docker build -t rasberry_perception:tensorrtdeepsort .
##
## Save with:
##   docker save rasberry_perception:tensorrtdeepsort | gzip >  rasberry_perception-tensorrtdeepsort.tar.gz
##
## Run with:
##   docker run --network host --gpus all --name tensorrt_backend --rm -it rasberry_perception:tensorrtdeepsort
##

FROM rasberry_perception:jetson


RUN DEBIAN_FRONTEND=noninteractive apt-get update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ninja-build protobuf-compiler libprotobuf-dev python  python3-venv

# Copy ModularMOT as is private. Can be accessed here https://github.com/SAGARobotics/PerceptionApps
COPY PerceptionApps /PerceptionApps

WORKDIR /
RUN git clone https://github.com/rjwb1/deep_sort.git

#TEMP Copy Weights. Acquire from rbelshaw@sagarobotics.com 
COPY mars_sb_14.pb /mars_sb_14.pb
COPY yolov4-416.onnx /PerceptionApps/ModularMOT/modularmot/models/yolov4-416.onnx

# set up environment variables
ENV PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV LLVM_CONFIG="/llvm-7.0.1.src/llvm_build_dir/bin/llvm-config"
ENV llvm='/llvm-7.0.1.src/llvm_build_dir/bin/llvm-lit'


#Tensorflow Dependencies
RUN DEBIAN_FRONTEND=noninteractive apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran 
RUN apt-get -y install python3-pip
RUN pip3 --no-cache-dir install -U pip testresources setuptools==49.6.0 

## Install backend in virtual Python3 environment
RUN python3.6 -m venv modularmot_venv --clear --system-site-packages && \
    . modularmot_venv/bin/activate && \
    pip install --no-cache-dir rospkg && \
    pip install --global-option=build_ext --global-option="-I/usr/local/cuda/include" --global-option="-L/usr/local/cuda/lib64" pycuda && \
    pip3 install --no-cache-dir -U numpy==1.19.4 future==0.18.2 mock==3.0.5 h5py==2.10.0 keras_preprocessing==1.1.1 keras_applications==1.0.8 gast==0.2.2 futures protobuf pybind11 && \
    pip install --no-cache-dir numpy && \
    pip install --no-cache-dir cython && \
    pip install --no-cache-dir cython-bbox && \
    pip3 install --no-cache-dir --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v45 tensorflow==1.15.4+nv20.12 && \   
    pip install --no-cache-dir scikit-build && \
    pip install --no-cache-dir opencv-python && \
    pip install --no-cache-dir scipy && \
    pip install --no-cache-dir scikit-learn==0.22

ENV PYTHONPATH "${PYTHONPATH}:/PerceptionApps/ModularMOT:/deep_sort"

WORKDIR /

RUN wget http://releases.llvm.org/7.0.1/llvm-7.0.1.src.tar.xz && \
    tar -xvf llvm-7.0.1.src.tar.xz 
WORKDIR /llvm-7.0.1.src/llvm_build_dir
RUN cmake ../ -DCMAKE_BUILD_TYPE=Release -DLLVM_TARGETS_TO_BUILD="ARM;X86;AArch64" && \
    make -j8 && \
    make install
WORKDIR /llvm-7.0.1.src/llvm_build_dir/bin

WORKDIR / 
RUN . modularmot_venv/bin/activate && \
pip install --no-cache-dir llvmlite==0.31.0 && \
pip install --no-cache-dir numba==0.48

WORKDIR /PerceptionApps/ModularMOT/modularmot/plugins
RUN make

WORKDIR /

# Docker clean-up
RUN rm -rf /var/lib/apt/lists/*

## Create entry point for image (default entry point looks for a start_backend.sh script that describes how to launch the backend)
COPY start_backend.sh .
CMD ["/bin/bash", "-c"]
