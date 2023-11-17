#!/usr/bin/env python
from foliated_problem import FoliatedProblem, FoliatedIntersection
from manipulation_foliations_and_intersections import ManipulationFoliation, ManipulationIntersection
from jiaming_task_planner import MTGTaskPlanner, MDPTaskPlanner, MTGTaskPlannerWithGMM, MDPTaskPlannerWithGMM, GMM
from jiaming_helper import convert_joint_values_to_robot_trajectory, convert_joint_values_to_robot_state, get_no_constraint, construct_moveit_constraint, make_mesh

import sys
import rospy
import rospkg


if __name__ == "__main__":

    rospack = rospkg.RosPack()
    
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')

    # load the gmm
    gmm_dir_path = package_path + '/computed_gmms_dir/dpgmm/'
    gmm = GMM()
    gmm.load_distributions(gmm_dir_path)

    # load the foliated problem
    loaded_foliated_problem = FoliatedProblem.load(ManipulationFoliation, ManipulationIntersection, package_path + "/check")

    # load it into the task planner.
    # task_planner = MTGTaskPlanner()
    # task_planner = MDPTaskPlanner()
    task_planner = MTGTaskPlannerWithGMM(gmm)

    task_planner.reset_task_planner()

    task_planner.load_foliated_problem(loaded_foliated_problem)

    task_planner.set_start_and_goal(
        (0,0),
        ManipulationIntersection(action='start', motion=[[0.38, -1.28, 1.52, 0.35, 1.81, 1.47, 0.04, 0.04]]),
        (1,1),
        ManipulationIntersection(action='start', motion=[[0.38, -1.28, 1.52, 0.35, 1.81, 1.47, 0.04, 0.04]])
    )

    task_sequence = task_planner.generate_task_sequence()

    for task in task_sequence:
        print "-------------------"
        print "start configuration"
        print(task.start_configuration)
        print "goal configuration"
        print(task.goal_configuration)
        print "constraints"
        print task.manifold_detail.foliation.foliation_name
        print task.manifold_detail.co_parameter_index
        print task.manifold_detail.foliation.constraint_parameters
        print "action on goal"
        print task.next_motion
