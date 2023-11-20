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


if __name__ == "__main__":

    rospy.init_node('main_pipeline_node', anonymous=True)

    rospack = rospkg.RosPack()
    
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')

    # load the foliated problem
    loaded_foliated_problem = FoliatedProblem.load(ManipulationFoliation, ManipulationIntersection, package_path + "/check")

    # load the gmm
    gmm_dir_path = package_path + '/computed_gmms_dir/dpgmm/'
    gmm = GMM()
    gmm.load_distributions(gmm_dir_path)

    # load it into the task planner.
    task_planner = MTGTaskPlanner()
    # task_planner = MDPTaskPlanner()
    # task_planner = MTGTaskPlannerWithGMM(gmm)
    # task_planner = MDPTaskPlannerWithGMM(gmm)

    # initialize the motion planner
    motion_planner = MoveitMotionPlanner()
    motion_planner.prepare_planner()

    # initialize the visualizer
    visualizer = MoveitVisualizer()

    # initialize the foliated planning framework
    foliated_planning_framework = FoliatedPlanningFramework(task_planner=task_planner, motion_planner=motion_planner)

    # set the visualizer
    foliated_planning_framework.setVisualizer(visualizer)

    # set the foliated problem
    foliated_planning_framework.setFoliatedProblem(loaded_foliated_problem)

    # set the start and goal
    foliated_planning_framework.setStartAndGoal(
        0, 0,
        ManipulationIntersection(action='start', motion=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]),
        0, 2,
        ManipulationIntersection(action='goal', motion=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
    )

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






















    # task_planner.reset_task_planner()

    # task_planner.load_foliated_problem(loaded_foliated_problem)

    # task_planner.set_start_and_goal(
    #     (0,0),
    #     ManipulationIntersection(action='start', motion=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]),
    #     (0,2),
    #     ManipulationIntersection(action='start', motion=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
    # )

    # task_sequence = task_planner.generate_task_sequence()

    # for task in task_sequence:
    #     print "-------------------"
    #     print "start configuration"
    #     print(task.start_configuration)
    #     print "goal configuration"
    #     print(task.goal_configuration)
    #     print "constraints"
    #     print task.manifold_detail.foliation.foliation_name
    #     print task.manifold_detail.co_parameter_index
    #     print task.manifold_detail.foliation.constraint_parameters
    #     print "action on goal"
    #     print task.next_motion
    
    #     # plan the motion
    #     success_flag, motion_plan_result = motion_planner.plan(
    #         task.start_configuration, 
    #         task.goal_configuration, 
    #         task.manifold_detail.foliation.constraint_parameters, 
    #         task.next_motion
    #     )

    #     break

    # motion_planner.shutdown_planner()