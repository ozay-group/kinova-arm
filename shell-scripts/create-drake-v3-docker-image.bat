:: create-drake-docker-container.bat
:: Description: create the docker image for the project
::   

:: Pull the latest public Docker image for Drake
docker pull robotlocomotion/drake:latest

:: Docker build the image
docker build -t drake-image-v3 docker/drake_v3/