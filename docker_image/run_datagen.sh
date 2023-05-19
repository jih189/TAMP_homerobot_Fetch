#!/bin/bash
set -ex
docker run -v $PWD/../:/root/catkin_ws/src/jiaming_manipulation \
	-e DISPLAY=":1" \
	-e QT_X11_NO_MITSHM=1 \
	-e XAUTHORITY \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	--ipc=host \
	--gpus all \
	--network=datagen \
	-p 8884:8884 \
	--privileged=true \
	-v /etc/localtime:/etc/localtime:ro \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -p 19994:19994 -it constraindatagen-ubuntu18 bash
