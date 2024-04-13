#!/bin/bash

EXP_NAME="Apr12_UR5_maze" # the name of the experiment, this will shown in the results script
MODEL_NAME="UR5" # UR5 | Fetch
GMM_NAME="dpgmm_6dof_127" # dpgmm_7dof_262 | dpgmm_6dof_443 | dpgmm_6dof_186 | dpgmm_7dof_1310
PARALLEL_TASK_PLANNERS=true # set to true if you want to evaluate multiple task planners in parallel (very costly careful)
IMAGE_NAME="rss2024_rebuttal_exp:apr12" # image name for the evaluation container you commited before

PLANNING_TIME=4.0
NUMBER_OF_TASKS=50
MAX_ATTEMPT_TIME=50

# List of problems to evaluate, should be same as the problems in the pre_generated_problems folder
declare -a problems=( 
#    "open_door_g_100"
    "maze_g_100"
#    "open_bottle_g_80"
#    "shelf_g_100"
#    "open_drawer_g_100"
#    "slide_cup_g_200"
#    "pour_water_g_250"
)

declare -a task_planners=(
    "MTGTaskPlannerWithGMM"
    "MTGTaskPlannerWithAtlas"
    "DynamicMTGTaskPlannerWithGMM"
    "DynamicMTGPlannerWithAtlas"
    "ALEFTaskPlanner"
    "MTGTaskPlanner"
)


if [ "$MODEL_NAME" = "UR5" ]; then
    MOVEIT_CONFIG="ur5_moveit_config"
else
    MOVEIT_CONFIG="fetch_moveit_config"
fi

DOCKER_NAME_PREFIX=$(date +%s)

# The evaluation script will run the evaluation for each problem in the list in background
# You can monitor the progress by checking the logs of the docker containers or use the jupyter scripts in the results folder
# The docker container will automatically stop after the evaluation is done but you need to delete them manually
if [ "$PARALLEL_TASK_PLANNERS" = true ]; then
    for task_planner in "${task_planners[@]}"
    do
        for problem in "${problems[@]}"
        do
            docker run --name ${DOCKER_NAME_PREFIX}_${MODEL_NAME}_${problem}_${task_planner} -d -e DISPLAY=":0" \
                -e QT_X11_NO_MITSHM=1 \
                -e XAUTHORITY \
                -e NVIDIA_DRIVER_CAPABILITIES=all \
                -e EXP_NAME=${EXP_NAME} \
                -e ROBOT_TYPE=${MODEL_NAME} \
                -e PLANNING_TIME=${PLANNING_TIME} \
                --ipc=host \
                --gpus all \
                --privileged=true \
                -v /etc/localtime:/etc/localtime:ro \
                -v /dev/video0:/dev/video0 \
                -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" ${IMAGE_NAME} \
                /bin/bash -c "source /root/catkin_ws/devel/setup.bash && \
                roslaunch ${MOVEIT_CONFIG} fake_move_group.launch & \
                (sleep 10; source /root/catkin_ws/devel/setup.bash; rosrun task_planner planner_evaluation.py --n ${NUMBER_OF_TASKS} --t ${MAX_ATTEMPT_TIME} --model ${MODEL_NAME} --problem ${problem} --gmm ${GMM_NAME} --task_planner ${task_planner})" &
        done
    done
else
    task_planners_csv=$(IFS=,; echo "${task_planners[*]}")
    for problem in "${problems[@]}"
    do
        docker run --name ${DOCKER_NAME_PREFIX}_${problem}_all_planners -d -e DISPLAY=":0" \
            -e QT_X11_NO_MITSHM=1 \
            -e XAUTHORITY \
            -e NVIDIA_DRIVER_CAPABILITIES=all \
            -e EXP_NAME=${EXP_NAME} \
            -e ROBOT_TYPE=${MODEL_NAME} \
            -e PLANNING_TIME=${PLANNING_TIME} \
            --ipc=host \
            --gpus all \
            --privileged=true \
            -v /etc/localtime:/etc/localtime:ro \
            -v /dev/video0:/dev/video0 \
            -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" ${IMAGE_NAME} \
            /bin/bash -c "source /root/catkin_ws/devel/setup.bash && \
            roslaunch ${MOVEIT_CONFIG} fake_move_group.launch & \
            (sleep 10; source /root/catkin_ws/devel/setup.bash; rosrun task_planner planner_evaluation.py --n ${NUMBER_OF_TASKS} --t ${MAX_ATTEMPT_TIME} --model ${MODEL_NAME} --problem ${problem} --gmm ${GMM_NAME} --task_planner ${task_planners_csv})" &
    done
fi
