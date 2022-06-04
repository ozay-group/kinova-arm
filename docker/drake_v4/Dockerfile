# Dockerfile
# Description:
#   This Dockerfile creates a container which can allow you to develop Drake applications.
#   It should also include:
#   - OpenCV
#   - Open3d
#   - AprilTags

FROM robotlocomotion/drake:latest

WORKDIR /root

# Install Git
RUN apt-get update
RUN apt-get --yes install git
RUN apt-get --yes install python3-pip
RUN pip3 install ipywidgets

## Install OpenCV ##

# Install curl
RUN apt-get install -y \
    wget \
    build-essential \
    cmake \
    unzip \
    pkg-config \
    python3-dev \
    python3-opencv \
    libopencv-dev \ 
    pyrealsense2 \
    ffmpeg  \
    libtbb2 libtbb-dev \ 
    libjpeg-dev \ 
    libpng-dev \ 
    libtiff-dev \ 
    libgtk2.0-dev \ 
    python3-pycurl \
    python-pycurl \ 
    libatlas-base-dev \
    gfortran \
    webp \ 
    qt5-default \
    libvtk6-dev \ 
    zlib1g-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev 

RUN apt-get --yes install libgstreamer1.0-dev  \
    libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa \
    gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio

# Install Open CV - Warning, this takes absolutely forever
RUN mkdir -p ~/opencv && cd ~/opencv && \
    wget https://github.com/opencv/opencv/archive/4.5.0.zip && \
    unzip 4.5.0.zip && \
    rm 4.5.0.zip && \
    mv opencv-4.5.0 OpenCV && \
    cd OpenCV && \
    mkdir build && \ 
    cd build && \
    cmake \
    -DWITH_QT=ON \ 
    -DWITH_OPENGL=ON \ 
    -DFORCE_VTK=ON \
    -DWITH_TBB=ON \
    -DWITH_GDAL=ON \
    -DWITH_XINE=ON \
    -DBUILD_EXAMPLES=ON .. && \
    make -j2 && \
    make install && \ 
    ldconfig

## Install open3d ##
RUN pip3 install open3d
RUN pip3 install --upgrade numpy
RUN pip3 install --upgrade scipy

## Install AprilTags and RealSense drivers
RUN pip3 install dt-apriltags pyrealsense2

## Clone Relevant Repositories ##

# Clone the Kinova Repository Into the Working Directory
RUN git clone https://github.com/Kinovarobotics/ros_kortex.git
RUN cp -r ros_kortex /opt/drake/share/drake

# Clone kinova_drake
# TODO - as this repository will likely be developed alongside the current one
#           this should probably be cloned locally and then copied over
RUN git clone https://github.com/kwesiRutledge/kinova_drake.git
RUN cp -r kinova_drake /opt/drake/share/drake

RUN cd kinova_drake && git remote add vincekurtz https://github.com/vincekurtz/kinova_drake.git

# Clone Exploration Repo + Install Kortex API via .whl file
#RUN git clone https://github.com/kwesiRutledge/OzayGroupExploration.git
#RUN cd OzayGroupExploration && python3 -m pip install #kortex_api-2.3.0.post34-py3-none-any.whl

# Install kortex wheel
COPY kortex_api-2.3.0.post34-py3-none-any.whl kortex_api-2.3.0.post34-py3-none-any.whl
RUN python3 -m pip install kortex_api-2.3.0.post34-py3-none-any.whl

# Copy instead of clone (optional)
COPY . /root/kinova-arm

