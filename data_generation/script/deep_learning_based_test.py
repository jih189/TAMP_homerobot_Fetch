#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from trajectory_generation import TrajectoryGenerator

def main():
    rospy.init_node('deep_learning_based_path_planner_test')

    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    path_planner_tester = TrajectoryGenerator(moveit_commander)

    # set the path planner id
    path_planner_tester.set_path_planner_id('DLBIRRTConfigDefault')

    env_num = 1

    obstacle_meshes = path_planner_tester.generate_random_mesh(env_num)
    pointcloud = path_planner_tester.setObstaclesInScene(obstacle_meshes)

    # randonly generate two valid points and path the solution between them
    plan_result, sampled_trajectory  = path_planner_tester.generateValidTrajectory()

    # publish the pointcloud of the obstacle
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        path_planner_tester.show_point_cloud(pointcloud)
        rate.sleep()

    # clear the planning scene.
    path_planner_tester.cleanPlanningScene()

if __name__ == '__main__':
    main()