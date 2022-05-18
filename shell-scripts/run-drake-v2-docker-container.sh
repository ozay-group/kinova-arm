# run-drake-docker-container.sh
# Description:
#   

IP1=35.3.126.205

docker run -td --name drake-container2 \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -p 7000:7000 \
    --network="host" \
    drake-image-v2

