# Dockerfile
# Description:
#   This Dockerfile creates a container which can allow you to develop Drake applications.

FROM robotlocomotion/drake:latest

WORKDIR /root

# Install Git
RUN apt update
RUN apt --yes install git
RUN apt --yes install python3-pip
RUN pip3 install ipywidgets

## Clone Relevant Repositories ##

# Clone the Kinova Repository Into the Working Directory
RUN git clone https://github.com/Kinovarobotics/ros_kortex.git
RUN cp -r ros_kortex /opt/drake/share/drake

# Clone the Kinova API Repository into the root
RUN git clone https://github.com/Kinovarobotics/kortex.git

# Clone kinova_drake
RUN git clone https://github.com/kwesiRutledge/kinova_drake.git
RUN cp -r kinova_drake /opt/drake/share/drake

# Clone Exploration Repo + Install Kortex API via .whl file
RUN git clone https://github.com/kwesiRutledge/OzayGroupExploration.git
RUN cd OzayGroupExploration && python3 -m pip install kortex_api-2.3.0.post34-py3-none-any.whl