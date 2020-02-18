xhost +local:docker

docker run -it\
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/lilinjun/Programs/robotics:/robotics"\
    --volume="/dev/input:/dev/input:rw"\
    --net=host\
    linjun/racecar\
    bash

