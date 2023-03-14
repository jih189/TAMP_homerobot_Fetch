#!/bin/bash
set -ex
docker run  -v $PWD/shared:/shared -e DISPLAY=":1" \
    -e QT_X11_NO_MITSHM=1 \
	-e XAUTHORITY \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	--ipc=host \
	--gpus all \
	--net=host \
    --privileged=true \
	-v /etc/localtime:/etc/localtime:ro \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -p 19997:19997 -it coppeliasim-ubuntu18 "$@"