#!/usr/bin/env python
import os

def select_robot_model():
    print "Please select a robot:"
    print "1. Fetch"
    print "2. UR5"
    selection = int(raw_input("Enter the robot you wish to select: "))
    return selection

selected_roobt_model = select_robot_model()

if selected_roobt_model == 1:
    os.environ['ROBOT_TYPE'] = 'FETCH'
elif selected_roobt_model == 2:
    os.environ['ROBOT_TYPE'] = 'UR5'

if os.environ.get('ROBOT_TYPE', False):
    print("ROBOT_TYPE already set to: {}".format(os.environ['ROBOT_TYPE']))
else:
    os.environ['ROBOT_TYPE'] = 'FETCH'

from foliated_base_class import FoliatedProblem, FoliatedIntersection
from manipulation_foliations_and_intersections import (
    ManipulationFoliation,
    ManipulationIntersection,
)
from foliated_planning_framework import FoliatedPlanningFramework
from jiaming_GMM import GMM
from jiaming_task_planner import (
    MTGTaskPlanner,
    ALEFTaskPlanner,
    MTGTaskPlannerWithGMM,
    MTGTaskPlannerWithAtlas,
)
from jiaming_motion_planner import MoveitMotionPlanner
from jiaming_visualizer import MoveitVisualizer
from jiaming_helper import INIT_JOINT_NAMES, INIT_JOINT_POSITIONS, INIT_ACTIVE_JOINT_POSITIONS
import rospy
import rospkg
import pickle
from moveit_msgs.msg import RobotState

def select_gmm_from_directory(directory):
    gmms = [d for d in os.listdir(directory) if os.path.isdir(os.path.join(directory, d))]
    print "Please select a GMM from the list:"
    for i, gmm in enumerate(gmms):
        print "{}: {}".format(i + 1, gmm)
    selection = int(raw_input("Enter the number of the GMM you wish to select: "))
    return gmms[selection - 1]

def select_problem_from_directory(directory):
    problems = [d for d in os.listdir(directory) if os.path.isdir(os.path.join(directory, d))]
    print "Please select a problem from the list:"
    for i, problem in enumerate(problems):
        print "{}: {}".format(i + 1, problem)
    selection = int(raw_input("Enter the number of the problem you wish to select: "))
    return problems[selection - 1]

def select_planner():
    print "Please select a planner:"
    print "1. MTGTaskPlanner"
    print "2. ALEFTaskPlanner"
    print "3. MTGTaskPlannerWithGMM"
    print "4. MTGTaskPlannerWithAtlas"
    selection = int(raw_input("Enter the number of the planner you wish to select: "))
    return selection

def set_start_goal():
    start_manifold = int(raw_input("Enter the number of the start manifold you wish to set: "))
    goal_manifold = int(raw_input("Enter the number of the goal manifold you wish to set: "))
    return start_manifold, goal_manifold

if __name__ == "__main__":
    rospy.init_node("main_pipeline_node", anonymous=True)

    rospack = rospkg.RosPack()

    # Get the path of the desired package
    package_path = rospack.get_path("task_planner")
    if selected_roobt_model == 1:
        os.environ['ROBOT_TYPE'] = 'FETCH'
        problems_directory = os.path.join(package_path, "problems/pre_generated_probs/fetch")
    elif selected_roobt_model == 2:
        os.environ['ROBOT_TYPE'] = 'UR5'
        problems_directory = os.path.join(package_path, "problems/pre_generated_probs/ur5")
    else:
        print "Invalid robot model selected"
        exit()
    
    joint_state_name = INIT_JOINT_NAMES
    joint_state_position = INIT_JOINT_POSITIONS
    default_motion = INIT_ACTIVE_JOINT_POSITIONS
    
    trajectory_directory = os.path.join(package_path, "problems/trajectorys")
    selected_problem = select_problem_from_directory(problems_directory)
    problem_file_path = os.path.join(problems_directory, selected_problem)

    # load the foliated problem
    loaded_foliated_problem = FoliatedProblem.load(
        ManipulationFoliation, ManipulationIntersection, problem_file_path
    )

    # load the gmm
    gmms_directory = os.path.join(package_path, "computed_gmms_dir")
    gmm_name = select_gmm_from_directory(gmms_directory)
    gmm_dir_path = os.path.join(gmms_directory, gmm_name) + "/"
    
    gmm = GMM()
    gmm.load_distributions(gmm_dir_path)

    # initialize the motion planner
    motion_planner = MoveitMotionPlanner()

    # # get the current robot state
    robot_state = RobotState()
    robot_state.joint_state.name = joint_state_name
    robot_state.joint_state.position = joint_state_position

    # initialize the foliated planning framework, and set the task planner and motion planner
    foliated_planning_framework = FoliatedPlanningFramework()
    foliated_planning_framework.setMotionPlanner(motion_planner)

    # load it into the task planner.
    
    planner_selection = select_planner()
    if planner_selection == 1:
        task_planner = MTGTaskPlanner()
    elif planner_selection == 2:
        task_planner = ALEFTaskPlanner()
    elif planner_selection == 3:
        task_planner = MTGTaskPlannerWithGMM(gmm)
    elif planner_selection == 4:
        task_planner = MTGTaskPlannerWithAtlas(
            gmm, motion_planner.move_group.get_current_state()
        )
    else:
        print "Invalid planner selected"
        exit()

    foliated_planning_framework.setTaskPlanner(task_planner)
    foliated_planning_framework.setMaxAttemptTime(30)

    # initialize the visualizer
    visualizer = MoveitVisualizer()
    visualizer.prepare_visualizer(
        motion_planner.move_group.get_active_joints(), motion_planner.robot
    )

    # set the visualizer
    foliated_planning_framework.setVisualizer(visualizer)

    # set the foliated problem
    foliated_planning_framework.setFoliatedProblem(loaded_foliated_problem)

    start, goal = set_start_goal()
    # set the start and goal
    foliated_planning_framework.setStartAndGoal(
        0,
        start,
        ManipulationIntersection(
            action="start",
            motion=[default_motion],
            active_joints=motion_planner.move_group.get_active_joints(),
        ),
        0,
        goal,
        ManipulationIntersection(
            action="goal",
            motion=[default_motion],
            active_joints=motion_planner.move_group.get_active_joints(),
        ),
    )

    # foliated_planning_framework.setStartAndGoal(
    #     0, 10,
    #     ManipulationIntersection(action='start', motion=[[ 0.38, -1.28, 1.51, 0.35, 1.81, 1.47, 0.0]], active_joints=motion_planner.move_group.get_active_joints()),
    #     0, 11,
    #     ManipulationIntersection(action='goal', motion=[[ 0.38, -1.28, 1.51, 0.35, 1.81, 1.47, 0.0]], active_joints=motion_planner.move_group.get_active_joints())
    # )

    # solve the problem
    found_solution, solution_trajectory = foliated_planning_framework.solve()

    if found_solution:
        print("found solution")
        # visualize the solution
        with open(trajectory_directory + '/solution_trajectory.pkl', 'wb') as f:
            pickle.dump(solution_trajectory, f)
        foliated_planning_framework.visualizeSolutionTrajectory(solution_trajectory)
        
        
    else:
        print("no solution found")

    # shutdown the planning framework
    foliated_planning_framework.shutdown()
