:: Run bash in docker and set display variable to host IP for X11
FOR /F "tokens=3" %%g IN ('wsl ip route list default') do (SET IP1=%%g)

docker start drake-container4
docker exec -ti -e DISPLAY=%IP1%:0 drake-container4 bash