#!/usr/bin/env python
from foliated_base_class import FoliatedProblem, FoliatedIntersection
from manipulation_foliations_and_intersections import ManipulationFoliation, ManipulationIntersection
from foliated_planning_framework import FoliatedPlanningFramework
from jiaming_task_planner import MTGTaskPlanner, MDPTaskPlanner, MTGTaskPlannerWithGMM, MDPTaskPlannerWithGMM, GMM
from jiaming_motion_planner import MoveitMotionPlanner
from jiaming_visualizer import MoveitVisualizer

import sys
import rospy
import rospkg

import networkx as nx


if __name__ == "__main__":

    rospy.init_node('main_pipeline_node', anonymous=True)

    rospack = rospkg.RosPack()
    
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')

    # load the foliated problem
    loaded_foliated_problem = FoliatedProblem.load(ManipulationFoliation, ManipulationIntersection, package_path + "/check")

    # load the gmm
    gmm_dir_path = package_path + '/computed_gmms_dir/dpgmm/'
    # gmm_dir_path = package_path + '/computed_gmms_dir/gmm/'
    gmm = GMM()
    gmm.load_distributions(gmm_dir_path)

    # load it into the task planner.
    # task_planner = MTGTaskPlanner()
    # task_planner = MDPTaskPlanner()
    task_planner = MTGTaskPlannerWithGMM(gmm)
    # task_planner = MDPTaskPlannerWithGMM(gmm)

    # initialize the motion planner
    motion_planner = MoveitMotionPlanner()
    motion_planner.prepare_planner()

    # initialize the visualizer
    visualizer = MoveitVisualizer()

    visualizer.prepare_visualizer(motion_planner.move_group.get_active_joints(), motion_planner.robot)

    # initialize the foliated planning framework
    foliated_planning_framework = FoliatedPlanningFramework(task_planner=task_planner, motion_planner=motion_planner)

    foliated_planning_framework.setMaxAttemptTime(5)
    
    # set the visualizer
    foliated_planning_framework.setVisualizer(visualizer)

    # set the foliated problem
    foliated_planning_framework.setFoliatedProblem(loaded_foliated_problem)

    # set the start and goal
    foliated_planning_framework.setStartAndGoal(
        0, 0,
        ManipulationIntersection(action='start', motion=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], active_joints=motion_planner.move_group.get_active_joints()),
        0, 14,
        ManipulationIntersection(action='goal', motion=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], active_joints=motion_planner.move_group.get_active_joints())
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
        print "found solution"
        # visualize the solution
        foliated_planning_framework.visualizeSolutionTrajectory(solution_trajectory)
    else:
        print "no solution found"

    # shutdown the planning framework
    foliated_planning_framework.shutdown()