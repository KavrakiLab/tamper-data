DEST_DIR=/home/robot/catkin_ws/src

docker run --gpus all -it -v /tmp/.X11-unix:/tmp/.X11-unix \
                          -v $PWD/dataset/:$DEST_DIR/dataset \
                          -v $PWD/scripts/:$DEST_DIR/scripts \
                          -e DISPLAY=$DISPLAY tamper/replicate-ros-bag /bin/bash