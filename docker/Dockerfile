# Dockerfile
# Description:
#   This Dockerfile creates a container which can allow you to develop Drake applications.

FROM robotlocomotion/drake:latest

WORKDIR /root

# Install Git
RUN apt update
RUN apt --yes install git
RUN apt install python3-pip
RUN pip3 install ipywidgets

# Clone the Kinova Repository Into the Working Directory
RUN git clone https://github.com/Kinovarobotics/ros_kortex.git
RUN cp -r ros_kortex /opt/drake/share/drake

# Clone kinova_drake
RUN git clone https://github.com/vincekurtz/kinova_drake.git
RUN cp -r kinova_drake /opt/drake/share/drake