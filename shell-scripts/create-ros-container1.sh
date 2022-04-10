# create-ros-container1.sh
# Description:
#   Creates the docker container for my ROS experiments.
-e

# Pull the most recent example of ROS
docker pull ros:latest

# build Docker container and run the container
docker build -t crazyswarm-image docker/ros-v1/

# Shut down any previously running containers and create a new one
docker rm -f crazyswarm-container
docker run -td --name crazyswarm-container \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    crazyswarm-image