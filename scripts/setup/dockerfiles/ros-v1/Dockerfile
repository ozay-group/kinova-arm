# Dockerfile
# Description:
#   This Dockerfile creates a ROS container that I hope can be used to do crazyswarm stuff.
#   Written by Kwesi Rutledge on Mar. 10, 17:36.

FROM ros:noetic-robot

WORKDIR /root

# Install Git
RUN apt-get update
RUN apt-get --yes install git
RUN apt-get --yes install python3-pip
RUN pip3 install ipywidgets

## Install Crazyswarm
# RUN export CSW_PYTHON=python3 # This command does not stay in memory, but a Docker variable will!
ENV CSW_PYTHON=python3
RUN echo ${CSW_PYTHON}
ENV DEBIAN_FRONTEND=noninteractive

# Install the dependencies
RUN apt-get install -y ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-tf-conversions ros-${ROS_DISTRO}-joy
RUN apt-get install -y libpcl-dev libusb-1.0-0-dev
RUN apt-get install -y swig lib${CSW_PYTHON}-dev
RUN ${CSW_PYTHON} -m pip install pytest numpy PyYAML matplotlib scipy

# Extra dependencies that I've found
RUN ${CSW_PYTHON} -m pip install PyQt5 vispy

# Install the optional dependencies
RUN apt-get install -y ffmpeg
RUN ${CSW_PYTHON} -m pip install ffmpeg-python

# Clone the Crazyswarm git repository
RUN git clone https://github.com/USC-ACTLab/crazyswarm.git

# RUN the build script
# RUN source /opt/ros/${ROS_DISTRO}/setup.sh

# RUN cd ~/crazyswarm \
#     && ./build.sh

## Clone Relevant Repositories ##

# Clone Exploration Repo + Install Kortex API via .whl file
RUN git clone https://github.com/kwesiRutledge/OzayGroupExploration.git