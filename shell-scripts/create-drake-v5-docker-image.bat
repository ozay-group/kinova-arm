:: create-drake-v4-docker-image.bat
:: Description: create the docker image for the project
::   

:: Pull the latest public Docker image for Drake
docker pull robotlocomotion/drake:latest

:: Docker build the image
docker build -t drake-image-v5 -f docker/drake_v5/Dockerfile .