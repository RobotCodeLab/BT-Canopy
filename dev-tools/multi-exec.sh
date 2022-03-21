#!/bin/sh

container_name="btwatcher-container"

# check if docker container is running using docker ps
if [ $(docker ps | grep $container_name | wc -l) -eq 0 ]; then
    echo "Container $container_name is not yet running"
    kill -INT $$
fi


# set num_tabs to the user's choice (default is 1)
num_tabs=${1:-1}

# if num_tabs is not a number or is less than 0, or greater than 9, exit
if ! [[ $num_tabs =~ ^[0-9]+$ ]] || [ $num_tabs -lt 1 ] || [ $num_tabs -gt 9 ]; 
then
    echo "Invalid number of tabs. Please enter a number between 1 and 9."
else

    for ((i=1; i<=$num_tabs; i++))
    do
        gnome-terminal --tab -- docker exec -it $container_name /bin/bash
    done

fi


