# create-docker-container.sh
# Description:
#   

# Pull the latest public Docker Container for Drake
docker pull robotlocomotion/drake:latest

# Docker build and run the container
docker build -t drake-image-v0 docker/

docker run -td --name drake-container \
    -p 7000:7000 \
    drake-image-v0

