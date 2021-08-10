# create-drake-docker-container.sh
# Description:
#   

# Pull the latest public Docker Container for Drake
docker pull robotlocomotion/drake:latest

# Docker build and run the container
docker build -t drake-image-v2 docker/drake/