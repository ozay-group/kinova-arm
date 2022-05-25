# run-drake-docker-v2-container.sh
# Description:
#   This script was meant to enable users of Mac OS X to:
#   - spin up a container using the image saved as drake-image-v2
#   - incorporate X11 forwarding from the container to the Mac (requires that you run `xhost +` in the XQuartz terminal)

if [[ $(uname) == 'Darwin' ]] ; then
    export IP1=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
else
    export IP1=35.3.126.205
fi

docker run -td --name drake-container2 \
    -e DISPLAY=$IP1:0 \
    -e XAUTHORITY=/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --privileged \
    --network="host" \
    drake-image-v2

