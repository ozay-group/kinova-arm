#!/bin/bash
# create-drake-v4-docker-image.sh
# Description: create the docker image for the project
#   

# Pull the latest public Docker Container for Drake
docker pull robotlocomotion/drake:latest

# Docker build and run the container
docker build -t drake-image-v4 -f docker/drake_v4/Dockerfile .