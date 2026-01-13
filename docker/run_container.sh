DATA_DIR="/media/$USER/T73/"
ROS_WS_DIR="/home/$USER/Research/ros_workspaces/lio_sam_ws"

docker run --init -it \
    --name="lio_sam" \
    --shm-size=2gb \
    --net="host" \
    --privileged \
    --gpus="all" \
    --workdir="/home/$USER/lio_sam_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/tmp/.Xauthority" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-$USER" \
    --env="USER_ID=$(id -u)" \
    --env="GROUP_ID=$(id -g)" \
    --volume="$ROS_WS_DIR:/home/$USER/lio_sam_ws" \
    --volume="$DATA_DIR:/home/$USER/data" \
    --volume="/home/$USER/.bash_aliases:/home/$USER/.bash_aliases" \
    --volume="/home/$USER/.ssh:/home/$USER/.ssh:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTHORITY:/tmp/.Xauthority:ro" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="/etc/timezone:/etc/timezone:ro" \
    --volume /tmp/runtime-$USER:/tmp/runtime-$USER \
    lio_sam  \
    /bin/bash