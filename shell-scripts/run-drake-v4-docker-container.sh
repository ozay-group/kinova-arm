#!/bin/bash
# run-drake-docker-v4-container.sh
# Description:
#   This script was meant to enable users of Mac OS X and Linux to:
#   - spin up a container using the image saved as drake-image-v4
#   - bind the project directory into the container
#   - incorporate X11 forwarding from the container to the host
# WARNING: Currently untested on macos and linux

if [[ $(uname) == 'Darwin' ]] ; then
    #For Mac OS X X11 forwarding requires that you run `xhost +` in the XQuartz terminal
    
    export IP1=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')

    docker run -td --name drake-container4 \
        --mount type=bind,source="$PWD",target="/root/kinova-arm" \
        -e DISPLAY=$IP1:0 \
        -e XAUTHORITY=/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -p 7001:7000 \
        --network="host" \
        drake-image-v4

elif [[ $(uname) == 'Linux' ]] ; then
    # Using the lab laptop (I hope)
    export IP1=

    # Running the container in a somewhat dangerous mode (privileged and with access to ALL DEVICES because we are sharing /dev
    # with the container).
    docker run -td --name drake-container4 \
        --mount type=bind,source="$PWD",target="/root/kinova-arm" \
        -e DISPLAY=$IP1:0 \
        -e XAUTHORITY=/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --device /dev:/dev \
        --network="host" \
        --privileged \
        drake-image-v4

else
    echo "Unrecognized OS. For Windows use the provide .bat script"
fi



