:: run-drake-docker-v5-container.bat
:: Description:
::   This script was meant to enable users of Windows to:
::   - spin up a container using the image saved as drake-image-v5
::   - bind the project directory into the container
::   - incorporate X11 forwarding from the container to the host Windows OS
::      - tested with vcXsrv
::      - you should not need to disable access control
::
:: Modify the variable gurobi_lic to mount your gurobi license file into the container

:: Hacky batch for fetching the appropriate IP address for the X11 server
:: TODO - is there an easy way to get the IP without WSL (although WSL is required for docker)
FOR /F "tokens=3" %%g IN ('wsl ip route list default') do (SET IP1=%%g)

set source_dir=%~dp0\..
set gurobi_lic=C:\Users\thinko\.gurobi_licenses_docker\gurobi.lic

:: Run docker image with X11 socket
docker run -td --name drake-container5 ^
    --mount type=bind,source="%source_dir%",target="/root/kinova-arm" ^
    --volume=%gurobi_lic%:/opt/gurobi/gurobi.lic:ro ^
    -e DISPLAY=%IP1%:0 ^
    -p 7001:7000 ^
    drake-image-v5