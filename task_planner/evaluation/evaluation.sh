#!/bin/bash

EXP_NAME="default" # the name of the experiment, this will shown in the results script

if [ "$EXP_NAME" = "default" ]; then
    EXP_NAME=$(date +%d-%m-default)
fi

MODEL_NAME="UR5" # FETCH | UR5
MOVEIT_CONFIG="ur5_moveit_config" # fetch_moveit_config | ur5_moveit_config
GMM_NAME="dpgmm_6dof_443" # dpgmm_7dof_262 | dpgmm_7dof_1310 | dpgmm_6dof_443 | dpgmm_6dof_186

DOCKER_NAME_PREFIX="manipulation_apr6_ur5" # assign a unique prefix for each docker container
IMAGE_NAME="rss2024_rebuttal_exp:apr6_v2" # image name for the evaluation container you commited before

# List of problems to evaluate, should be same as the problems in the pre_generated_problems folder
declare -a problems=( 
   "open_door_g_100"
   "open_drawer_g_100"
   "maze_g_100"
    "slide_cup_g_200"
   "shelf_g_100"
   "open_bottle_g_80"
    "pour_water_g_250"
)

# The evaluation script will run the evaluation for each problem in the list in background
# You can monitor the progress by checking the logs of the docker containers or use the jupyter scripts in the results folder
# The docker container will automatically stop after the evaluation is done but you need to delete them manually

for problem in "${problems[@]}"
do
    docker run --name ${DOCKER_NAME_PREFIX}_${problem} -d -e DISPLAY=":0" \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e EXP_NAME=${EXP_NAME} \
        -e ROBOT_TYPE=${MODEL_NAME} \
        --ipc=host \
        --gpus all \
        --privileged=true \
        -v /etc/localtime:/etc/localtime:ro \
        -v /dev/video0:/dev/video0 \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" ${IMAGE_NAME} \
        /bin/bash -c "source /root/catkin_ws/devel/setup.bash && \
        roslaunch ${MOVEIT_CONFIG} fake_move_group.launch & \
        (sleep 10; source /root/catkin_ws/devel/setup.bash; rosrun task_planner planner_evaluation.py --model ${MODEL_NAME} --problem ${problem} --gmm ${GMM_NAME})" &
done