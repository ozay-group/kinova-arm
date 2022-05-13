# run-drake-docker-v2-container.sh
# Description:
#   This script was meant to enable users of Mac OS X to:
#   - spin up a container using the image saved as drake-image-v2
#   - incorporate X11 forwarding from the container to the Mac (requires that you run `xhost +` in the XQuartz terminal)

export IP1=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')

docker run -td --name drake-container2 \
    -e DISPLAY=$IP1:0 \
    -e XAUTHORITY=/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -p 7001:7000 \
    --network="host" \
    drake-image-v2

