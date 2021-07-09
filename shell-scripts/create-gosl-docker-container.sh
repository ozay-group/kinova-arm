# create-gosl-docker-container.sh
# Description:
#   Creates the docker container for my gosl experiments.

# Pull the most recent example of gosl
docker pull gosl/gosl

# build Docker container and run the container
docker build -t gosl-image-v1 docker/gosl/

docker run -td --name gosl-container gosl-image-v1
