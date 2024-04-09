#!/usr/bin/env python
import os

from foliated_base_class import FoliatedProblem, FoliatedIntersection
from manipulation_foliations_and_intersections import (
    ManipulationFoliation,
    ManipulationIntersection,
)
from foliated_planning_framework import FoliatedPlanningFramework
from jiaming_GMM import GMM
from jiaming_task_planner import (
    MTGTaskPlanner,
    MTGTaskPlannerWithGMM,
    MTGTaskPlannerWithAtlas,
    ALEFTaskPlanner,
    DynamicMTGTaskPlannerWithGMM,
    DynamicMTGPlannerWithAtlas,
)
from jiaming_motion_planner import MoveitMotionPlanner
from jiaming_helper import INIT_ACTIVE_JOINT_POSITIONS
import rospy
import rospkg
from tqdm import tqdm
import json
import uuid
import time
import redis
import socket
import argparse

"""
On the side, for each foliation problem, we need to provide the possible start and goal manifolds.
The pipeline for the evaluation of the foliated planning framework.

Add a new function to the foliated planning framework:
    evaluation(): this function will solve the problem and return the success flag and planning time of task sequence generation, motion planning, and updating time.

folaition_problem = load the foliated problem.
start_and_goal_list = generate different a list of start and goal for the foliation problem.
for planner in planner_list:
    planning_time = 0
    success_count = 0
    For start, goal in start_and_goal_list:
        foliated_planning_framework.setStartAndGoal(start, goal)
        found_solution, solution_trajectory = foliated_planning_framework.solve()
        if found_solution:
            success_count += 1
            planning_time += foliated_planning_framework.planning_time()

    print "planner: ", planner, " success rate: ", success_count / len(start_and_goal_list), " average planning time: ", planning_time / len(start_and_goal_list)
    save the result to a file.
"""

def select_robot_model():
    print "Please select a robot:"
    print "1. Fetch"
    print "2. UR5"
    selection = int(raw_input("Enter the number of the problem you wish to select: "))
    if selection == 1:
        return "FETCH"
    elif selection == 2:
        return "UR5"

def select_problem_from_directory(directory):
    problems = [d for d in os.listdir(directory) if os.path.isdir(os.path.join(directory, d))]
    print "Please select a problem from the list:"
    for i, problem in enumerate(problems):
        print "{}: {}".format(i + 1, problem)
    selection = int(raw_input("Enter the number of the problem you wish to select: "))
    return problems[selection - 1]

def select_gmm_from_directory(directory):
    gmms = [d for d in os.listdir(directory) if os.path.isdir(os.path.join(directory, d))]
    print "Please select a GMM from the list:"
    for i, gmm in enumerate(gmms):
        print "{}: {}".format(i + 1, gmm)
    selection = int(raw_input("Enter the number of the GMM you wish to select: "))
    return gmms[selection - 1]

if __name__ == "__main__":
    
    redis_host = "rss.redis.cache.windows.net"
    redis_port = 6379
    redis_password = "AQouyZ83AaEt7lBhgpLCAhBLip3ygGHfZAzCaM3IaqI="
    redis_db = 0
    redis_retry_on_timeout = True
    redis_socket_timeout = 5
    
    def init_redis_connection():
        for i in range(5):
            try:
                r = redis.StrictRedis(
                    host=redis_host,
                    port=redis_port,
                    password=redis_password,
                    db=redis_db,
                    socket_timeout=redis_socket_timeout,
                    retry_on_timeout=redis_retry_on_timeout
                )
                r.ping() 
                return r
            except redis.ConnectionError as e:
                if i < 4: 
                    print("Redis connection failed. Retrying...")
                    time.sleep(1)
                else:
                    print("Failed to connect to Redis after several attempts: {}".format(e))
                    return None

    redis_connection = init_redis_connection()
    
    number_of_tasks = 50
    max_attempt_time = 50

    ########################################
    
    
    parser = argparse.ArgumentParser(description='Foliated planning framework evaluation.')
    parser.add_argument('-model', '--model', help='The model of the robot to use.')
    parser.add_argument('-prob', '--problem', help='The name of the foliation problem to use.')
    parser.add_argument('-gmm', '--gmm_name', help='The name of the GMM to use.')
    args = parser.parse_args()
    

    rospy.init_node("evaluation_node", anonymous=True)

    task_name = rospy.get_param("~task_name", "")

    rospack = rospkg.RosPack()

    # Get the path of the desired package
    package_path = rospack.get_path("task_planner")

    if args.model:
        selected_roobt_model = args.model
        print("Using provided model: {}".format(selected_roobt_model))
    else:
        selected_roobt_model = select_robot_model()
        
    if selected_roobt_model == "FETCH":
        os.environ['ROBOT_TYPE'] = 'FETCH'
        problems_directory = os.path.join(package_path, "problems/pre_generated_probs/fetch")
    elif selected_roobt_model == "UR5":
        os.environ['ROBOT_TYPE'] = 'UR5'
        problems_directory = os.path.join(package_path, "problems/pre_generated_probs/ur5")
    else:
        print "Invalid robot model selected"
        exit()
    
    results_directory = os.path.join(package_path, "problems/results")
    gmms_directory = os.path.join(package_path, "computed_gmms_dir")
    
    if args.problem:
        selected_problem = args.problem
        print("Using provided problem: {}".format(selected_problem))
    else:
        selected_problem = select_problem_from_directory(problems_directory)
        
    print selected_problem
    problem_file_path = os.path.join(problems_directory, selected_problem)

    # load the foliated problem
    loaded_foliated_problem = FoliatedProblem.load(
        ManipulationFoliation, ManipulationIntersection, problem_file_path
    )

    task_uuid = str(uuid.uuid4())
    task_timestamp = time.time()
    hostname = os.environ.get('EXP_NAME', 'unknown') 
    
    # load the gmm
    # gmm_name = "dpgmm"
    # gmm_dir_path = package_path + "/computed_gmms_dir/" + gmm_name + "/"
    # gmm_dir_path = package_path + '/computed_gmms_dir/gmm/'
    
    if args.gmm_name:
        gmm_name = args.gmm_name
        print("Using provided GMM: {}".format(gmm_name))
    else:
        gmm_name = select_gmm_from_directory(gmms_directory)
        
    gmm_dir_path = os.path.join(gmms_directory, gmm_name) + "/"
    print("Using GMM: " + gmm_name)

    gmm = GMM()
    gmm.load_distributions(gmm_dir_path)
    
    # set the result file path
    result_file_path = os.path.join(results_directory, task_name + selected_problem + "_" + str(task_timestamp) + "_" + task_uuid + ".json")
    result_key = hostname + ":" + selected_problem + ":" + gmm_name + ":" + str(task_timestamp) + "_" + task_uuid


    # Set the path for the config file
    config_file_path = os.path.join(problems_directory, task_name + selected_problem + "_config.json")

    # sampled random start and goal
    sampled_start_and_goal_list = []

    # check if the config file exists
    if os.path.isfile(config_file_path):
        # load the sampled_start_and_goal_list from the file
        with open(config_file_path, 'r') as config_file:
            sampled_start_and_goal_list = json.load(config_file)
        print("Loaded start and goal configurations from the existing config file")
    else:
        # sample a new start and goal list and save to the config file
        sampled_start_and_goal_list = [
            loaded_foliated_problem.sampleStartAndGoal() for _ in range(number_of_tasks)
        ]
        with open(config_file_path, 'w') as config_file:
            json.dump(sampled_start_and_goal_list, config_file)
        print("Generated and saved new start and goal configurations to the config file")

    # initialize the motion planner
    motion_planner = MoveitMotionPlanner()
    motion_planner.prepare_planner()

    # initialize the foliated planning framework
    foliated_planning_framework = FoliatedPlanningFramework()
    foliated_planning_framework.setMotionPlanner(motion_planner)
    foliated_planning_framework.setMaxAttemptTime(max_attempt_time)
    # set the foliated problem
    foliated_planning_framework.setFoliatedProblem(loaded_foliated_problem)

    # load it into the task planner.
    task_planners = [
        MTGTaskPlannerWithGMM(gmm),
        MTGTaskPlannerWithAtlas(gmm, motion_planner.move_group.get_current_state()),
        DynamicMTGTaskPlannerWithGMM(gmm, planner_name_="DynamicMTGTaskPlannerWithGMM_25.0", threshold=25.0),
        DynamicMTGPlannerWithAtlas(gmm, motion_planner.move_group.get_current_state(), planner_name_="DynamicMTGPlannerWithAtlas_50.0", threshold=25.0),
        DynamicMTGPlannerWithAtlas(gmm, motion_planner.move_group.get_current_state(), planner_name_="DynamicMTGPlannerWithAtlas_50.0", threshold=50.0),
        ALEFTaskPlanner(),
        MTGTaskPlanner(),
    ]


    with open(result_file_path, "w") as result_file:
        for task_planner in task_planners:
            print("=== Evaluate task planner ", task_planner.planner_name, " ===")

            foliated_planning_framework.setTaskPlanner(task_planner)

            for task_info in tqdm(sampled_start_and_goal_list):
                start, goal = task_info

                # set the start and goal
                foliated_planning_framework.setStartAndGoal(
                    start[0],
                    start[1],
                    ManipulationIntersection(
                        action="start",
                        motion=[INIT_ACTIVE_JOINT_POSITIONS],
                        active_joints=motion_planner.move_group.get_active_joints(),
                    ),
                    goal[0],
                    goal[1],
                    ManipulationIntersection(
                        action="goal",
                        motion=[INIT_ACTIVE_JOINT_POSITIONS],
                        active_joints=motion_planner.move_group.get_active_joints(),
                    ),
                )

                # solve the problem
                (
                    success_flag,
                    task_planning_time,
                    motion_planning_time,
                    updating_time,
                    solution_length,
                    num_attempts,
                    total_solve_time,
                    set_start_and_goal_time,
                    task_graph_size,
                    current_task_graph_size,
                ) = foliated_planning_framework.evaluation()
                
                task_timestamp = time.time()

                if success_flag:
                    result_data = {
                        "planner_name": task_planner.planner_name,
                        "start": start,
                        "goal": goal,
                        "success": "true",
                        "total_planning_time": total_solve_time,
                        "task_planning_time": task_planning_time,
                        "motion_planning_time": motion_planning_time,
                        "set_start_and_goal_time": set_start_and_goal_time,
                        "updating_time": updating_time,
                        "solution_length": solution_length,
                        "num_attempts": num_attempts,
                        "task_uuid": task_uuid,
                        "task_timestamp": task_timestamp,
                        "gmm_name": gmm_name,
                        "task_graph_size": task_graph_size,
                        "current_task_graph_size": current_task_graph_size,
                    }
 
 
                    if redis_connection:
                        try:
                            redis_connection.rpush(result_key, json.dumps(result_data))
                        except Exception as e:
                            print("Failed to publish data to Redis: {}".format(e))

                    try:
                        json.dump(result_data, result_file)
                        result_file.write("\n")
                        result_file.flush()
                    except Exception as e:
                        print("Failed to write data to file: {}".format(e))
 
                else:
                    task_timestamp = time.time()
                    result_data = {
                        "planner_name": task_planner.planner_name,
                        "start": start,
                        "goal": goal,
                        "success": "false",
                        "total_planning_time": -1,
                        "task_planning_time": -1,
                        "motion_planning_time": -1,
                        "set_start_and_goal_time": -1,
                        "updating_time": -1,
                        "solution_length": -1,
                        "num_attempts": -1,
                        "task_uuid": task_uuid,
                        "task_timestamp": task_timestamp,
                        "gmm_name": gmm_name,
                        "task_graph_size": task_graph_size,
                        "current_task_graph_size": current_task_graph_size,
                    }


                    if redis_connection:
                        try:
                            redis_connection.rpush(result_key, json.dumps(result_data))
                        except Exception as e:
                            print("Failed to publish data to Redis: {}".format(e))

                    try:
                        json.dump(result_data, result_file)
                        result_file.write("\n")
                        result_file.flush()
                    except Exception as e:
                        print("Failed to write data to file: {}".format(e))
        del task_planner

    # shutdown the planning framework
    foliated_planning_framework.shutdown()