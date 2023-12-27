#!/usr/bin/env python
# from experiment_scripts.experiment_helper import Experiment, Manifold, Intersection
from foliated_base_class import FoliatedProblem
from manipulation_foliations_and_intersections import ManipulationFoliation, ManipulationIntersection
import os
import sys
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
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import yaml
import tqdm

def convert_pose_stamped_to_matrix(pose_stamped):
    pose_matrix = transformations.quaternion_matrix([pose_stamped.pose.orientation.w, 
                                                    pose_stamped.pose.orientation.x, 
                                                    pose_stamped.pose.orientation.y, 
                                                    pose_stamped.pose.orientation.z])
    pose_matrix[0, 3] = pose_stamped.pose.position.x
    pose_matrix[1, 3] = pose_stamped.pose.position.y
    pose_matrix[2, 3] = pose_stamped.pose.position.z
    return pose_matrix

def visualize_environment_and_placements(env_pose, env_mesh_path, feasible_placements, manipulated_object_mesh_path):
    """
    Visualize both intermediate placements and obstacles
    """
    problem_publisher = rospy.Publisher('/problem_visualization_marker_array', MarkerArray, queue_size=5)
    
    marker_array = MarkerArray()

    # visualize the obstacle
    obstacle_marker = Marker()
    obstacle_marker.header.frame_id = "base_link"
    obstacle_marker.header.stamp = rospy.Time.now()
    obstacle_marker.ns = "obstacle"
    obstacle_marker.id = 0
    obstacle_marker.type = Marker.MESH_RESOURCE
    obstacle_marker.action = Marker.ADD
    obstacle_marker.pose = env_pose.pose
    obstacle_marker.scale = Point(1, 1, 1)
    obstacle_marker.color = ColorRGBA(0.5, 0.5, 0.5, 1)
    obstacle_marker.mesh_resource = "package://task_planner/mesh_dir/" + os.path.basename(env_mesh_path)
    marker_array.markers.append(obstacle_marker)

    # visualize the placements
    for i, placement in enumerate(feasible_placements):
        object_marker = Marker()
        object_marker.header.frame_id = "base_link"
        object_marker.header.stamp = rospy.Time.now()
        object_marker.ns = "placement"
        object_marker.id = i + 1
        object_marker.type = Marker.MESH_RESOURCE
        object_marker.action = Marker.ADD
        object_marker.pose = msgify(geometry_msgs.msg.Pose, placement)
        object_marker.scale = Point(1, 1, 1)
        object_marker.color = ColorRGBA(0.5, 0.5, 0.5, 1)
        object_marker.mesh_resource = "package://task_planner/mesh_dir/" + os.path.basename(manipulated_object_mesh_path)
        marker_array.markers.append(object_marker)
        
    problem_publisher.publish(marker_array)


def initialize_robot_and_scene():
    """
    Initialize the robot and the scene
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('create_experiment_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(0.5)
    scene.clear()

    move_group = moveit_commander.MoveGroupCommander("arm")
    rospy.wait_for_service("/compute_ik")
    compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

    return robot, scene, move_group, compute_ik_srv
    

def set_initial_joint_state(joint_state_dict):
    """
    Set the initial joint state
    :param joint_state_dict: dictionary with joint names as keys and joint positions as values
    """
    joint_state_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = list(joint_state_dict.keys())
    joint_state.position = list(joint_state_dict.values())

    rate = rospy.Rate(10)
    while joint_state_publisher.get_num_connections() < 1:
        rate.sleep()
    joint_state_publisher.publish(joint_state)
    
    
def get_position_difference_between_poses(pose_1_, pose_2_):
    '''
    Get the position difference between two poses.
    pose_1_ and pose_2_ are both 4x4 numpy matrices.
    '''
    return np.linalg.norm(pose_1_[:3, 3] - pose_2_[:3, 3])

def gaussian_similarity(distance, max_distance, sigma=0.01):
    """
    Calculate the similarity score using Gaussian function.
    distance: the distance between two configurations
    sigma: the sigma of the Gaussian function
    max_distance: the maximum distance between two configurations
    The score is between 0 and 1. The larger the score, the more similar the two configurations are.
    If sigma is heigher, the scope of the Gaussian function is wider.
    """
    if distance == 0: # when the distance is 0, the score should be 1
        return 1.0

    # Calculate the similarity score using Gaussian function
    score = np.exp(-(distance**2) / (2 * sigma**2))
    max_score = np.exp(-(max_distance**2) / (2 * sigma**2))
    score = (score - max_score) / (1 - max_score)

    if score < 0.001:
        score = 0.0

    return score

# the function to prepare the sampler
def prepare_sampling_function(scene, env_pose, env_mesh_path):
    
    scene.add_mesh('env_obstacle', env_pose, env_mesh_path)

# the function to clear the sampler
def sampling_done_function(scene):
    scene.clear()

# define the sampling function
def slide_regrasp_sampling_function(co_parameters1, co_parameters2, manipulated_object_mesh_path):
    # co_parameters1 is the co-parameters for sliding foliation
    # co_parameters2 is the co-parameters for regrasping foliation
    # return a ManipulationIntersection class

    # randomly select a index for both co_parameters1 and co_parameters2
    selected_co_parameters1_index = random.randint(0, len(co_parameters1) - 1)
    selected_co_parameters2_index = random.randint(0, len(co_parameters2) - 1)

    # randomly sample a placement
    placement = co_parameters2[selected_co_parameters2_index]

    # randomly sample a grasp
    grasp = co_parameters1[selected_co_parameters1_index]

    # need to calculate the grasp pose in the base_link frame
    grasp_pose_mat = np.dot(placement, grasp)
    pre_grasp_pose_mat = np.dot(grasp_pose_mat, np.array([[1, 0 ,0, -0.05],
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

    return True, selected_co_parameters1_index, selected_co_parameters2_index, ManipulationIntersection(
        'release', 
        intersection_motion, 
        move_group.get_active_joints(), 
        placement,
        manipulated_object_mesh_path,
        convert_pose_stamped_to_matrix(env_pose),
        env_mesh_path
    )

def create_pose_stamped(pose_data):
    pose = PoseStamped()
    pose.header.frame_id = pose_data['frame_id']
    pose.pose.position.x = pose_data['position']['x']
    pose.pose.position.y = pose_data['position']['y']
    pose.pose.position.z = pose_data['position']['z']
    pose.pose.orientation.x = pose_data['orientation']['x']
    pose.pose.orientation.y = pose_data['orientation']['y']
    pose.pose.orientation.z = pose_data['orientation']['z']
    pose.pose.orientation.w = pose_data['orientation']['w']
    return pose

class FoliatedProblemBuilder(object):
    def __init__(self, package_path, parameters):
        self.package_path = package_path
        self.parameters = parameters
        self.env_mesh_path = package_path + parameters['env_mesh_path']
        self.manipulated_object_mesh_path = package_path + parameters['manipulated_object_mesh_path']
        self.grasp_poses_file = package_path + parameters['grasp_poses_file']
        self.env_pose = create_pose_stamped(parameters['env_pose'])
        self.table_top_pose = np.array(parameters['table_top_pose'])
        self.env_mesh = None
        self.collision_manager = None
        self.feasible_placements = []
        self.feasible_grasps = []

    def load_environment(self):
        self.env_mesh = trimesh.load_mesh(self.env_mesh_path)
        print(self.env_mesh_path)
        self.env_mesh.apply_transform(convert_pose_stamped_to_matrix(self.env_pose))
        self.collision_manager = trimesh.collision.CollisionManager()
        self.collision_manager.add_object('env', self.env_mesh)

    def find_feasible_placements(self, num_of_row, num_of_col, x_shift, y_shift, z_shift):
        for i in range(num_of_row):
            for j in range(num_of_col):
                obj_mesh = trimesh.load_mesh(self.manipulated_object_mesh_path)

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

                self.collision_manager.add_object('obj', obj_mesh)

                if not self.collision_manager.in_collision_internal():
                    self.feasible_placements.append(convert_pose_stamped_to_matrix(obj_pose))
                    
                self.collision_manager.remove_object('obj')

    def find_feasible_grasps(self, num_samples, rotated_matrix):
        loaded_array = np.load(self.grasp_poses_file)
        for ind in random.sample(list(range(len(loaded_array.files))), num_samples):
            self.feasible_grasps.append(np.dot(loaded_array[loaded_array.files[ind]], rotated_matrix))

    def create_foliations(self, sliding_sigma, orientation_tolerance, position_tolerance):
        # Calculate sliding similarity matrix
        different_matrix = np.zeros((len(self.feasible_grasps), len(self.feasible_grasps)))
        for i, grasp in enumerate(self.feasible_grasps):
            for j, grasp in enumerate(self.feasible_grasps):
                if i == j:
                    different_matrix[i, j] = 0
                different_matrix[i, j] = get_position_difference_between_poses(self.feasible_grasps[i], self.feasible_grasps[j])

        sliding_similarity_matrix = np.zeros((len(self.feasible_grasps), len(self.feasible_grasps)))
        max_distance = np.max(different_matrix)
        for i, grasp in enumerate(self.feasible_grasps):
            for j, grasp in enumerate(self.feasible_grasps):
                sliding_similarity_matrix[i, j] = gaussian_similarity(different_matrix[i, j], max_distance, sigma=sliding_sigma)

        # Build the foliations for both re-grasping and sliding
        foliation_regrasp = ManipulationFoliation(foliation_name='regrasp', 
                                                    constraint_parameters={
                                                        "frame_id": "base_link",
                                                        "is_object_in_hand": False,
                                                        "object_mesh_path": self.manipulated_object_mesh_path,
                                                        "obstacle_mesh": self.env_mesh_path,
                                                        "obstacle_pose": convert_pose_stamped_to_matrix(self.env_pose)
                                                    }, 
                                                    co_parameters=self.feasible_placements,
                                                    similarity_matrix=np.identity(len(self.feasible_placements)))

        foliation_slide = ManipulationFoliation(foliation_name='slide', 
                                                constraint_parameters={
                                                    "frame_id": "base_link", 
                                                    "is_object_in_hand": True,
                                                    'object_mesh_path': self.manipulated_object_mesh_path,
                                                    "obstacle_mesh": self.env_mesh_path,
                                                    "obstacle_pose": convert_pose_stamped_to_matrix(self.env_pose),
                                                    "reference_pose": self.table_top_pose,
                                                    "orientation_tolerance": orientation_tolerance,
                                                    "position_tolerance": position_tolerance
                                                }, 
                                                co_parameters=self.feasible_grasps,
                                                similarity_matrix=sliding_similarity_matrix)
        return foliation_regrasp, foliation_slide

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('task_planner')
    
    # Open Config
    with open(package_path + '/problems/maze.yaml', 'r') as yaml_file:
        config = yaml.safe_load(yaml_file)

    robot, scene, move_group, compute_ik_srv = initialize_robot_and_scene()
    set_initial_joint_state(config['initial_joint_state'])
    
    
    # Build Problem
    builder = FoliatedProblemBuilder(package_path, config['environment'])
    builder.load_environment()
    builder.find_feasible_placements(**config['placement_parameters'])
    builder.find_feasible_grasps(**config['grasp_parameters'])
    foliation_regrasp, foliation_slide = builder.create_foliations(**config['foliation_parameters'])
    
    # TODO: Sample Pipeline
    prepare_sampling_function(scene, builder.env_pose, builder.env_mesh_path)
    number_of_samples=500
    for i in tqdm(range(0, number_of_samples)):
        success_flag, co_parameter1_index, co_parameter2_index, sampled_intersection = self.slide_regrasp_sampling_function(foliation_slide.co_parameters, foliation_regrasp.co_parameters)

        if success_flag:
            # print 'co_parameter1_index: ' + str(co_parameter1_index) + ', co_parameter2_index: ' + str(co_parameter2_index)

            sampled_intersection.set_foliation_names_and_co_parameter_indexes(
                foliated_intersection.foliation1.foliation_name,
                co_parameter1_index,
                foliated_intersection.foliation2.foliation_name,
                co_parameter2_index
            )
            self.intersections.append(sampled_intersection)

            # inverse the intersection
            inversed_sampled_intersection = sampled_intersection.inverse()
            inversed_sampled_intersection.set_foliation_names_and_co_parameter_indexes(
                foliated_intersection.foliation2.foliation_name,
                co_parameter2_index,
                foliated_intersection.foliation1.foliation_name,
                co_parameter1_index
            )

            # append the inverse intersection
            self.intersections.append(inversed_sampled_intersection)

    sampling_done_function(scene)
    print "sampled " + str(self.intersections.__len__()) + " intersections bewteen foliations ", foliated_intersection.foliation1.foliation_name, " and ", foliated_intersection.foliation2.foliation_name
    
    # Visualization
    visualize_environment_and_placements(builder.env_pose, builder.env_mesh_path, builder.feasible_placements, builder.manipulated_object_mesh_path)

    
    
    
    # foliated_problem.save(package_path + task_params['save_path'])
    # loaded_foliated_problem = FoliatedProblem.load(ManipulationFoliation, ManipulationIntersection, package_path + task_params['save_path'])

    
    
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)