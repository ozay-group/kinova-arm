:: run-drake-docker-v3-container.bat
:: Description:
::   This script was meant to enable users of Windows to:
::   - spin up a container using the image saved as drake-image-v3
::   - incorporate X11 forwarding from the container to the host Windows OS
::      - tested with vcXsrv
::      - you should not need to disable access control
::

:: Hacky batch for fetching the appropriate IP address for the X11 server
FOR /F "tokens=3" %%g IN ('wsl ip route list default') do (SET IP1=%%g)

:: Run docker image with X11 socket
docker run -td --name drake-container3 ^
    -e DISPLAY=%IP1%:0 ^
    -v /tmp/.X11-unix:/tmp/.X11-unix ^
    --network="host" ^
    drake-image-v3