#!/usr/bin/env python3
"""
experiment helper
------------------
Here are the helper classes to save and load experiments. Thus, later the task planner
can use this code to load and construct the task graph.
"""
import os
import numpy as np
import sys
import rospkg
import json

class Manifold:
    """
    Manifold is the class describe all information of a constraint manifold in foliations.
    """

    def __init__(self, foliation_id_, manifold_id_, object_name_, object_mesh_, has_object_in_hand_):
        """
        Initialize the class instance.
        """
        self.foliation_id = foliation_id_
        self.manifold_id = manifold_id_
        self.object_name = object_name_
        self.object_mesh = object_mesh_
        self.has_object_in_hand = has_object_in_hand_

    def add_constraint(self, in_hand_pose_, constraint_pose_, orientation_constraint_, position_constraint_):
        """
        Add a constraint to the manifold.
        """
        if not self.has_object_in_hand:
            raise ValueError("The object is not in the hand, thus the constraint cannot be added.")
        self.in_hand_pose = in_hand_pose_
        self.constraint_pose = constraint_pose_
        self.orientation_constraint = orientation_constraint_
        self.position_constraint = position_constraint_

    def add_object_placement(self, object_pose_):
        """
        Add the object placement constraint to the manifold.
        """
        if self.has_object_in_hand:
            raise ValueError("The object is in the hand, thus the object placement constraint cannot be added.")
        self.object_pose = object_pose_

class Intersection:
    """
    Intersection describes the motion across two different manifolds.
    """
    def __init__(self, foliation_id_1_, manifold_id_1_, foliation_id_2_, manifold_id_2_, has_object_in_hand_, trajectory_motion_, in_hand_pose_, object_mesh_, object_name_):
        self.foliation_id_1 = foliation_id_1_
        self.manifold_id_1 = manifold_id_1_
        self.foliation_id_2 = foliation_id_2_
        self.manifold_id_2 = manifold_id_2_
        self.has_object_in_hand = has_object_in_hand_
        self.trajectory_motion =  trajectory_motion_
        self.in_hand_pose = in_hand_pose_ # if the object is not in hand, then in_hand_pose is the object placement pose.
        self.object_mesh = object_mesh_
        self.object_name = object_name_

class Experiment:
    """
    Experiment contains information of all manifolds and itersections.
    """
    def __init__(self, experiment_name_):
        self.experiment_name = experiment_name_
        self.manifolds = []
        self.intersections = []
        self.obstacle_mesh = None

    def add_manifold(self, manifold):
        self.manifolds.append(manifold)

    def add_intersection(self, intersection):
        self.intersections.append(intersection)

    def save(self, dir_name):
        """
        Save the experiment to a file.
        """
        # check if the directory exists
        # if so, then delete it
        if os.path.exists(dir_name):
            # delete the directory
            os.system("rm -rf " + dir_name)

        os.makedirs(dir_name)

        # Data to be saved
        experiment_data = {
            "experiment_name": "pick_and_place",      
        }

        # Save data to a JSON file
        with open(dir_name + '/meta_data.json', 'w') as file:
            json.dump(experiment_data, file)

    def load(self, dir_name):
        """
        Load the experiment from a file.
        """

    def print_experiment_data(self):
        """
        Print the experiment data.
        """


    

if __name__ == "__main__":
    # Create a experiment and save it to a file
    print("Create a experiment and save it to a file")
    rospack = rospkg.RosPack()
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')
    
    # check python version
    # experiment = Experiment("pick_and_place")

    # for pick and place manipulation, there are three foliations.
    # foliation 1(pre-grasp): the object is on the table and the robot hand is empty
    pre_grasp_manifold = Manifold(0, # foliation id
                                  0, # manifold id
                                  "mug", # object name
                                  "mug", # object mesh
                                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # in hand pose
                                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # constraint pose
                                  [0.0, np.pi, 0.0], # orientation constraint
                                  [0.0, 0.0, 0.0]) # position constraint
    # experiment.add_manifold(pre_grasp_manifold)
    # foliation 2(grasp): the object is grasped by the robot hand
    # foliation 3(post-grasp): the object is placed on the table and the robot hand is empty


    # experiment.save(package_path + "/experiment_dir/pick_and_place")
