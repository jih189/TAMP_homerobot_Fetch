# -*- coding: utf-8 -*-

import rospy
import os
import json
from moveit_commander import PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import MoveGroupAction
from utils import convert_pose_to_matrix, convert_matrix_to_pose_stamped

class ProblemManager(object):
    def __init__(self, robot, scene, move_group, compute_ik_srv, visualizer):
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.compute_ik_srv = compute_ik_srv
        self.visualizer = visualizer
        self.sampler = ProblemSampler(robot, move_group, compute_ik_srv)

    def setup(self):
        

    def teardown(self):
        self.scene.clear()

    def create_foliated_problem(self):

        return foliated_problem

    def save_problem(self, path, foliated_problem):
        with open(path, 'w') as f:
            json.dump(foliated_problem.to_dict(), f)

    def load_problem(self, path):
        with open(path, 'r') as f:
            problem_dict = json.load(f)
            foliated_problem = FoliatedProblem.from_dict(problem_dict)
            return foliated_problem

    def visualize_environment(self, env_pose, env_mesh_path, placements, object_mesh_path):
        self.visualizer.visualize_obstacle(env_pose, env_mesh_path)
        self.visualizer.visualize_placements(placements, object_mesh_path)
