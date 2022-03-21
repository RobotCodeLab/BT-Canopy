# BTWatcher

- `docker build -t btwatcher .`

- `rocker --x11 --nvidia --name btwatcher_container btwatcher`

- `rocker --x11 --devices /dev/dri/card0 -- --name btwatcher_container btwatcher`

- `. multi-exec.sh {1-9}` 

- `docker ps`
  
- `docker exec -it <container id> /bin/bash`

- `docker system prune`