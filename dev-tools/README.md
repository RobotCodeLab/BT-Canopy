# BTWatcher

Before docker building btwatcher, you will need to build [turtlebot-galactic](https://github.com/hodgespodge/ros-robot-docker-images/tree/main/turtlebot-galactic) found in [ros-robot-docker-images](https://github.com/hodgespodge/ros-robot-docker-images)

- `docker build -t btwatcher .` or `docker build --no-cache -t btwatcher .`

- `rocker --x11 --nvidia --name btwatcher-container btwatcher`

- `rocker --x11 --devices /dev/dri/card0 -- --name btwatcher-container btwatcher`

- `. multi-exec.sh {1-9}` 

- `docker ps`
  
- `docker exec -it <container id> /bin/bash`

- `docker system prune`