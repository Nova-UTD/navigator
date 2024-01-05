# Use Ubuntu as the base image
FROM ubuntu:latest

# Update package lists and install necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    protobuf-compiler \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory in the container
WORKDIR /app
RUN mkdir -p /test

# Install required Python packages
RUN pip3 install tensorflow pillow Cython lxml jupyter matplotlib contextlib2 tf_slim opencv-python object_detection protobuf==3.20

# Clone TensorFlow Models repository
#RUN git clone https://github.com/tensorflow/models.git

# Set environment variables
ENV PYTHONPATH="/app/models:${PYTHONPATH}"

# Compile protos and install object detection API
RUN apt-get update && \
    apt-get install -y git && \
    git clone https://github.com/tensorflow/models.git

ADD https://github.com/protocolbuffers/protobuf/releases/download/v25.1/protoc-25.1-linux-x86_64.zip models/research/
    #tar -xJC && \
    #make -C all && \
RUN apt-get install unzip
RUN cd models/research/ && \
	unzip protoc-25.1-linux-x86_64.zip -d protoc-25.1-linux-x86_64/ && \
    protoc object_detection/protos/*.proto --python_out=. 
    #&& \
    #pip3 install .
RUN apt-get install ffmpeg libsm6 libxext6  -y

# Adding files used locally
ADD test.sh models/research/
ADD detection.py models/research/object_detection/
ADD objDec.py models/research/object_detection/
ADD cabc30fc-e7726578.mov models/research/object_detection/

RUN cd models/research/ && \
	export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim

# Set up a default command or entrypoint if needed
WORKDIR ..
WORKDIR /app/models/research
CMD ["bash"]

