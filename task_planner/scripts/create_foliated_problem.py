#!/usr/bin/env python
# from experiment_scripts.experiment_helper import Experiment, Manifold, Intersection
from foliated_problem import FoliatedProblem, BaseFoliation, FoliatedIntersection, BaseIntersection

import sys
import copy
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.srv import GetJointWithConstraints, GetJointWithConstraintsRequest
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import Constraints, OrientationConstraint
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState
import tf.transformations as tf_trans
from ros_numpy import numpify, msgify
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Point32
import trimesh
from trimesh import transformations
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
import struct
# from manipulation_test.srv import *
import random
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
import json

def convert_pose_stamped_to_matrix(pose_stamped):
    pose_matrix = transformations.quaternion_matrix([pose_stamped.pose.orientation.w, 
                                                    pose_stamped.pose.orientation.x, 
                                                    pose_stamped.pose.orientation.y, 
                                                    pose_stamped.pose.orientation.z])
    pose_matrix[0, 3] = pose_stamped.pose.position.x
    pose_matrix[1, 3] = pose_stamped.pose.position.y
    pose_matrix[2, 3] = pose_stamped.pose.position.z
    return pose_matrix

if __name__ == "__main__":

    rospack = rospkg.RosPack()
    
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('create_experiment_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    # remove all objects from the scene, you need to give some time to the scene to update.
    rospy.sleep(0.5)
    scene.clear()
    
    move_group = moveit_commander.MoveGroupCommander("arm")
    rospy.wait_for_service("/compute_ik")
    compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

    # set initial joint state
    joint_state_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)

    # Create a JointState message
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'wrist_flex_joint', 'l_gripper_finger_joint', 'r_gripper_finger_joint']
    joint_state.position = [0.38, -1.28, 1.52, 0.35, 1.81, 1.47, 0.04, 0.04]

    rate = rospy.Rate(10)
    while(joint_state_publisher.get_num_connections() < 1): # need to wait until the publisher is ready.
        rate.sleep()
    joint_state_publisher.publish(joint_state)

    ########################################## create a foliated problem ##########################################

    # For the maze problem, we have two foliations:
    # 1. The foliation for sliding.
    # 2. The foliation for re-grasping.
    # find all co-parameter for both foliations

    table_top_pose = np.array([[1, 0, 0, 0.5],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.8],
                            [0, 0, 0, 1]])

    env_pose = PoseStamped()
    env_pose.header.frame_id = "base_link"
    env_pose.pose.position.x = 0.51
    env_pose.pose.position.y = 0.05
    env_pose.pose.position.z = -0.02
    env_pose.pose.orientation.x = 0
    env_pose.pose.orientation.y = 0
    env_pose.pose.orientation.z = 0.707
    env_pose.pose.orientation.w = 0.707

    env_mesh_path = package_path + "/mesh_dir/maze.stl"
    manipulated_object_mesh_path = package_path + '/mesh_dir/cup.stl'

    env_mesh = trimesh.load_mesh(env_mesh_path)
    env_mesh.apply_transform(convert_pose_stamped_to_matrix(env_pose))

    collision_manager = trimesh.collision.CollisionManager()
    collision_manager.add_object('env', env_mesh)

    # find all feasible placements as the co-parameter for re-grasping foliation
    num_of_row = 6
    num_of_col = 8
    x_shift = 0.56
    y_shift = 0.1
    z_shift = 0.8
    feasible_placements = []
    for i in range(num_of_row):
        for j in range(num_of_col):
            obj_mesh = trimesh.load_mesh(manipulated_object_mesh_path)

            obj_pose = PoseStamped()
            obj_pose.header.frame_id = "base_link"
            obj_pose.pose.position.x = i * 0.1 - num_of_row * 0.1 / 2 + x_shift
            obj_pose.pose.position.y = j * 0.1 - num_of_col * 0.1 / 2 + y_shift
            obj_pose.pose.position.z = z_shift
            obj_pose.pose.orientation.x = 0
            obj_pose.pose.orientation.y = 0
            obj_pose.pose.orientation.z = 0
            obj_pose.pose.orientation.w = 1

            obj_mesh.apply_transform(convert_pose_stamped_to_matrix(obj_pose))

            collision_manager.add_object('obj', obj_mesh)

            if not collision_manager.in_collision_internal():
                feasible_placements.append(convert_pose_stamped_to_matrix(obj_pose))
                
            collision_manager.remove_object('obj')

    # find all feasible grasps as the co-parameter for sliding foliation
    feasible_grasps = []

    loaded_array = np.load(package_path + "/mesh_dir/cup.npz")
    rotated_matrix = np.array([[1, 0, 0, -0.17],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])

    for ind in random.sample(list(range(len(loaded_array.files))), 40):
        feasible_grasps.append(np.dot(loaded_array[loaded_array.files[ind]], rotated_matrix)) # add the grasp poses in object frame

    ####################################################################################################################

    # define the intersection class
    class ManipulationIntersection(BaseIntersection):
        def __init__(self, action, motion):
            self.action = action
            self.motion = motion
        def inverse(self):
            if self.action == 'grasp':
                return ManipulationIntersection(action='release', motion=self.motion[::-1])
            else:
                return ManipulationIntersection(action='grasp', motion=self.motion[::-1])

        def save(self, file_path):
            # need to save the foliation name, co_parameter_index, action, motion
            foliation1_name, co_parameter1_index, foliation2_name, co_parameter2_index = self.get_foliation_names_and_co_parameter_indexes()
            
            intersection_data = {
                "foliation1_name": foliation1_name,
                "co_parameter1_index": co_parameter1_index,
                "foliation2_name": foliation2_name,
                "co_parameter2_index": co_parameter2_index,
                "action": self.action,
                "motion": [m.tolist() for m in self.motion]
            }

            # create a json file
            with open(file_path, "w") as json_file:
                json.dump(intersection_data, json_file)

        @staticmethod
        def load(file_path):

            with open(file_path, "r") as json_file:
                intersection_data = json.load(json_file)

            loaded_intersection = ManipulationIntersection(action=intersection_data.get("action"),
                                                motion=[np.array(m) for m in intersection_data.get("motion")])

            foliation1_name = intersection_data.get("foliation1_name")
            co_parameter1_index = intersection_data.get("co_parameter1_index")
            foliation2_name = intersection_data.get("foliation2_name")
            co_parameter2_index = intersection_data.get("co_parameter2_index")

            loaded_intersection.set_foliation_names_and_co_parameter_indexes(foliation1_name, co_parameter1_index, foliation2_name, co_parameter2_index)
            
            return loaded_intersection

    # define the foliation class
    class ManipulationFoliation(BaseFoliation):
        def save(self, dir_path):
            # save foliation name, constraint_parameters, co_parameters

            # if a value in constraint_parameters is a numpy array, convert it to list
            copy_constraint_parameters = copy.deepcopy(self.constraint_parameters)
            for key, value in copy_constraint_parameters.items():
                if isinstance(value, np.ndarray):
                    copy_constraint_parameters[key] = value.tolist()

            foliation_data = {
                "foliation_name": self.foliation_name,
                "constraint_parameters": copy_constraint_parameters,
                "co_parameters": [c.tolist() for c in self.co_parameters] # convert numpy array to list
            }

            # create a json file
            with open(dir_path + "/" + self.foliation_name + ".json", "w") as json_file:
                json.dump(foliation_data, json_file)

        @staticmethod
        def load(file_path):

            with open(file_path, "r") as json_file:
                foliation_data = json.load(json_file)

            # if a value in constraint_parameters is a list with 4x4 size, convert it to numpy array
            copy_constraint_parameters = copy.deepcopy(foliation_data.get("constraint_parameters"))
            for key, value in copy_constraint_parameters.items():
                if isinstance(value, list):
                    # check if the list can be converted to numpy array
                    try:
                        m = np.array(value)
                        # check if the matrix m is 4x4
                        if m.shape == (4, 4):
                            copy_constraint_parameters[key] = m
                    except:
                        pass

            return ManipulationFoliation(
                foliation_name=foliation_data.get("foliation_name"),
                constraint_parameters=copy_constraint_parameters,
                co_parameters=[np.array(c) for c in foliation_data.get("co_parameters")] # convert list to numpy array
            )

    # build the foliations for both re-grasping and sliding
    foliation_regrasp = ManipulationFoliation(foliation_name='regrasp', 
                                                constraint_parameters={
                                                    "frame_id": "base_link",
                                                    "object_mesh_path": manipulated_object_mesh_path,
                                                    "obstacle_mesh": env_mesh_path,
                                                    "obstacle_pose": convert_pose_stamped_to_matrix(env_pose)
                                                }, 
                                                co_parameters=feasible_placements)

    print "number of feasible placements: ", feasible_placements.__len__()

    foliation_slide = ManipulationFoliation(foliation_name='slide', 
                                            constraint_parameters={
                                                'frame_id': "base_link", 
                                                'object_mesh_path': manipulated_object_mesh_path,
                                                "obstacle_mesh": env_mesh_path,
                                                "obstacle_pose": convert_pose_stamped_to_matrix(env_pose),
                                                "reference_pose": table_top_pose,
                                                "orientation_tolerance": np.array([0.1, 0.1, 2*3.14]),
                                                "position_tolerance": np.array([2000, 2000, 0.05])
                                            }, 
                                            co_parameters=feasible_grasps)
    
    print "number of feasible grasps: ", feasible_grasps.__len__()

    def prepare_sampling_function():
        scene.add_mesh('env_obstacle', env_pose, env_mesh_path)

    def sampling_done_function():
        scene.clear()

    # define the sampling function
    def sampling_function(co_parameters1, co_parameters2):
        # co_parameters1 is the co-parameters for re-grasping foliation
        # co_parameters2 is the co-parameters for sliding foliation
        # return a ManipulationIntersection class

        # randomly select a index for both co_parameters1 and co_parameters2
        selected_co_parameters1_index = random.randint(0, len(co_parameters1) - 1)
        selected_co_parameters2_index = random.randint(0, len(co_parameters2) - 1)

        # randomly sample a placement
        placement = co_parameters1[selected_co_parameters1_index]

        # randomly sample a grasp
        grasp = co_parameters2[selected_co_parameters2_index]

        # need to calculate the grasp pose in the base_link frame
        grasp_pose_mat = np.dot(placement, grasp)
        pre_grasp_pose_mat = np.dot(grasp_pose_mat, np.array([[1, 0 ,0, -0.09],
                                                              [0, 1, 0, 0],
                                                              [0, 0, 1, 0],
                                                              [0, 0, 0, 1]]))

        # set the ik target pose
        ik_target_pose = PoseStamped()
        ik_target_pose.header.stamp = rospy.Time.now()
        ik_target_pose.header.frame_id = "base_link"
        ik_target_pose.pose = msgify(geometry_msgs.msg.Pose, grasp_pose_mat)

        ik_req = GetPositionIKRequest()
        ik_req.ik_request.group_name = "arm"
        ik_req.ik_request.avoid_collisions = True
        ik_req.ik_request.pose_stamped = ik_target_pose

        # set the robot state randomly
        random_moveit_robot_state = robot.get_current_state()
        random_position_list = list(random_moveit_robot_state.joint_state.position)
        for joint_name, joint_value in zip(move_group.get_joints(), move_group.get_random_joint_values()):
            random_position_list[random_moveit_robot_state.joint_state.name.index(joint_name)] = joint_value
        random_moveit_robot_state.joint_state.position = tuple(random_position_list)
        ik_req.ik_request.robot_state = random_moveit_robot_state
        
        ik_res = compute_ik_srv(ik_req)

        if not ik_res.error_code.val == MoveItErrorCodes.SUCCESS:
            return False, selected_co_parameters1_index, selected_co_parameters2_index, None

        # need to check the motion from grasp to pre-grasp
        moveit_robot_state = robot.get_current_state()
        moveit_robot_state.joint_state.position = ik_res.solution.joint_state.position

        move_group.set_start_state(moveit_robot_state)
        (planned_motion, fraction) = move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, pre_grasp_pose_mat)], 0.01, 0.0)

        if fraction < 0.97:
            return False, selected_co_parameters1_index, selected_co_parameters2_index, None

        intersection_motion = np.array([p.positions for p in planned_motion.joint_trajectory.points])

        return True, selected_co_parameters1_index, selected_co_parameters2_index, ManipulationIntersection(action='release', motion=intersection_motion)
        
    foliated_intersection = FoliatedIntersection(foliation_regrasp, foliation_slide, sampling_function, prepare_sampling_function, sampling_done_function)

    foliated_problem = FoliatedProblem("maze_task")
    foliated_problem.set_foliation_n_foliated_intersection([foliation_regrasp, foliation_slide], [foliated_intersection])
    foliated_problem.sample_intersections()
    ###############################################################################################################
    
    # save the foliated problem
    foliated_problem.save(package_path + "/check")

    # load the foliated problem
    loaded_foliated_problem = FoliatedProblem.load(ManipulationFoliation, ManipulationIntersection, package_path + "/check")

    # shutdown the moveit
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)