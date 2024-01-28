#!/usr/bin/env python
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
)
from jiaming_motion_planner import MoveitMotionPlanner
from jiaming_visualizer import MoveitVisualizer

import rospy
import rospkg
import pickle
from moveit_msgs.msg import RobotState


if __name__ == "__main__":
    rospy.init_node("main_pipeline_node", anonymous=True)

    rospack = rospkg.RosPack()

    # Get the path of the desired package
    package_path = rospack.get_path("task_planner")

    # load the foliated problem
    loaded_foliated_problem = FoliatedProblem.load(
        ManipulationFoliation, ManipulationIntersection, package_path + "/check"
    )

    # load the gmm
    gmm_dir_path = package_path + "/computed_gmms_dir/dpgmm/"
    # gmm_dir_path = package_path + '/computed_gmms_dir/gmm/'
    gmm = GMM()
    gmm.load_distributions(gmm_dir_path)

    # initialize the motion planner
    motion_planner = MoveitMotionPlanner()

    # get the current robot state
    start_robot_state = RobotState()
    start_robot_state.joint_state.name = ['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
    start_robot_state.joint_state.position = [0.36, -1.28, 1.51, 0.35, 1.81, 0.0, 1.47, 0.0]

    # initialize the foliated planning framework, and set the task planner and motion planner
    foliated_planning_framework = FoliatedPlanningFramework()
    foliated_planning_framework.setMotionPlanner(motion_planner, is_real_robot=True)

    # move the robot to the current robot state
    motion_planner.move_to_start_robot_state(start_robot_state)

    # load it into the task planner.
    # task_planner = MTGTaskPlanner()
    # task_planner = MTGTaskPlannerWithGMM(gmm)
    task_planner = MTGTaskPlannerWithAtlas(
          gmm, motion_planner.move_group.get_current_state()
    )  # need to provide the current robot state as the default robot state.
    foliated_planning_framework.setTaskPlanner(task_planner)
    foliated_planning_framework.setMaxAttemptTime(30)

    # initialize the visualizer
    visualizer = MoveitVisualizer()
    visualizer.prepare_visualizer(
        motion_planner.move_group.get_active_joints(), motion_planner.robot
    )

    # set the visualizer
    foliated_planning_framework.setVisualizer(visualizer)

    print("found solution")
    raw_input("Press Enter to visualize and execute the solution trajectory")
    # visualize the solution
    # foliated_planning_framework.visualizeSolutionTrajectory(solution_trajectory)

    motion_planner.move_to_start_robot_state(start_robot_state)

    # Loop to allow for repeated execution based on user input
    while True:
        with open('/root/catkin_ws/src/jiaming_manipulation/task_planner/problems/dev/open_drawer_1.pkl', 'rb') as f:
            solution_trajectory = pickle.load(f)

        motion_planner.execute(solution_trajectory)

        # Ask the user if they want to continue
        user_input = raw_input("Do you want to execute the trajectory again? (yes/no): ")
        if user_input.strip().lower() != "yes":
            break  # Exit the loop if the user does not want to continue


    # shutdown the planning framework
    foliated_planning_framework.shutdown()
