# create-gosl-docker-container.sh
# Description:
#   Creates the docker container for my gosl experiments.

# Pull the most recent example of gosl
docker pull julia:latest

# build Docker container and run the container
docker build -t julia-image-rantzer docker/julia/

# Shut down any previously running containers and create a new one
docker rm -f rantzer-container
docker run -td --name rantzer-container julia-image-rantzer
