# run-drake-docker-container.sh
# Description:
#   

IP1=$(ipconfig getifaddr en0)

docker run -td --name drake-container1 \
    -e DISPLAY=$IP1:0 \
    -e XAUTHORITY=/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/.Xauthority \
    -p 7000:7000 \
    -p 192.168.1.10:24:24 \
    --privileged \
    drake-image-v1

