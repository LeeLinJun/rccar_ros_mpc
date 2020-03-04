xhost +local:docker

docker run -it\
    --network host\
    --privileged\
    --env="DISPLAY" \
    --env="ROS_MASTER_URI=http://car01:11311"\
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/lilinjun/Programs/robotics:/robotics"\
    --volume="/dev/input:/dev/input:rw"\
    linjun/racecar\
    bash

