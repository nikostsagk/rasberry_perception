FROM rasberry_perception:base_gpu

# Meta information
MAINTAINER Raymond Kirk "rkirk@lincoln.ac.uk"

# Copy local mmdetection repo
COPY mmdetection /mmdetection
COPY model.pth /mmdetection/model.pth
COPY config.py /mmdetection/config.py

# Install dependancies and run python setup script
RUN bash -c "apt install -y python-pip && pip install rospkg"

# Currently manual edit is still required

CMD ["/bin/bash", "-c"]