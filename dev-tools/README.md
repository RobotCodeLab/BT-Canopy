# BTWatcher

Before docker building btwatcher, you will need to build [turtlebot-galactic](https://github.com/hodgespodge/ros-robot-docker-images/tree/main/turtlebot-galactic) found in [ros-robot-docker-images](https://github.com/hodgespodge/ros-robot-docker-images)

- `docker build -t btcanvas .` or `docker build --no-cache -t btcanvas .`

- `rocker --x11 --nvidia --name btcanvas-container btcanvas`

- `rocker --x11 --devices /dev/dri/card0  --name btcanvas-container btcanvas`

- `. multi-exec.sh {1-9}` 

- `docker ps`
  
- `docker exec -it <container id> /bin/bash`

- `docker system prune`