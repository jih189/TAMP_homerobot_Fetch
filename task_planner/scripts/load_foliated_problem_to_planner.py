#!/usr/bin/env python
from foliated_problem import FoliatedProblem, FoliatedIntersection
from manipulation_foliations_and_intersections import ManipulationFoliation, ManipulationIntersection
from jiaming_task_planner import MTGTaskPlanner, MDPTaskPlanner, MTGTaskPlannerWithGMM, MDPTaskPlannerWithGMM, GMM, ManifoldDetail, IntersectionDetail
from jiaming_helper import convert_joint_values_to_robot_trajectory, convert_joint_values_to_robot_state, get_no_constraint, construct_moveit_constraint, make_mesh

import sys
import rospy
import rospkg


if __name__ == "__main__":

    rospack = rospkg.RosPack()
    
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')

    # load the foliated problem
    loaded_foliated_problem = FoliatedProblem.load(ManipulationFoliation, ManipulationIntersection, package_path + "/check")

    # load it into the task planner.
    task_planner = MTGTaskPlanner()

    task_planner.reset_task_planner()

    task_planner.load_foliated_problem(loaded_foliated_problem)