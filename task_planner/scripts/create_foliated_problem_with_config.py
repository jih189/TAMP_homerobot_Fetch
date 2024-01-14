#!/usr/bin/env python
import os
import sys
import yaml
import trimesh
import numpy as np
import random
import rospkg
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf_trans

from sensor_msgs.msg import JointState
from manipulation_foliations_and_intersections import ManipulationFoliation
from foliated_base_class import FoliatedIntersection, FoliatedProblem
from jiaming_helper import create_pose_stamped, get_position_difference_between_poses, gaussian_similarity, \
    create_pose_stamped_from_raw, collision_check, convert_pose_stamped_to_matrix
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import MoveItErrorCodes
from manipulation_foliations_and_intersections import ManipulationIntersection
from ros_numpy import numpify, msgify
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class Config(object):
    def __init__(self, package_name):
        self.config_data = {}
        self.package_path = None

        self.load_config(package_name)

    def load_config(self, package_name):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('task_planner')
        self.package_path = package_path

        try:
            with open(self.package_path + '/problems/config.yaml', 'r') as yaml_file:
                config_base = yaml.safe_load(yaml_file)

            problem_path = config_base["path"]

            with open(self.package_path + problem_path, 'r') as yaml_file:
                self.config_data = yaml.safe_load(yaml_file)

            random_seed = self.config_data.get('task_parameters', {}).get('random_seed', None)
            if random_seed is not None:
                random.seed(random_seed)
                print("Random seed has been set to: " + str(random_seed))
            else:
                print("No random seed detected")

        except Exception as e:
            print("Unable to load config" + str(e))
            self.config_data = None

    def get(self, section, key=None, default=None):
        config_target = None
        if key:
            config_target = self.config_data.get(section, {}).get(key, default)
        else:
            config_target = self.config_data.get(section, {})
        return config_target

    def set(self, section, key, value):
        if section not in self.config_data:
            self.config_data[section] = {}
        self.config_data[section][key] = value


class FoliatedBuilder(object):
    def __init__(self, config):
        self.package_path = config.package_path
        self.env_mesh_path = self.package_path + config.get('environment', 'env_mesh_path')
        self.manipulated_object_mesh_path = self.package_path + config.get('environment',
                                                                           'manipulated_object_mesh_path')
        self.grasp_poses_file = self.package_path + config.get('environment', 'grasp_poses_file')
        self.env_pose = create_pose_stamped(config.get('environment', 'env_pose'))
        self.ref_pose = np.array(config.get('environment', 'ref_pose'))

        self.foliation_grasp = None
        self.foliation_placement_group = []
        self.sliding_similarity_matrix = None
        self.similarity_sigma = config.get('placement', 'similarity_sigma')
        self.env_mesh = None
        self.collision_manager = None
        self.feasible_placements = []
        self.feasible_grasps = []

        self.placement_parameters = config.get('placement')
        self.grasp_parameters = config.get('grasp')

        self.initialize()

    def initialize(self):
        self._load_environment()
        self._find_feasible_grasps(**self.grasp_parameters)
        self._calc_similarity_matrix(self.feasible_grasps)
        self._find_feasible_placements(self.placement_parameters)
        self._create_grasp_foliation()

    def _load_environment(self):
        self.env_mesh = trimesh.load_mesh(self.env_mesh_path)
        self.env_mesh.apply_transform(convert_pose_stamped_to_matrix(self.env_pose))
        self.collision_manager = trimesh.collision.CollisionManager()
        self.collision_manager.add_object('env', self.env_mesh)

    def _placement_rectangular(self, params):
        foliations = params["foliations"]
        for idx, foliation in enumerate(foliations):
            foliation_name = foliation["name"]
            reference_pose = np.array(foliation["reference_pose"])
            orientation_tolerance = foliation["orientation_tolerance"]
            position_tolerance = foliation["position_tolerance"]
            self._create_foliation(foliation_name, "base_link", reference_pose, orientation_tolerance,
                                   position_tolerance, self.feasible_grasps, self.sliding_similarity_matrix)

            size_row = foliation.get("size_row")
            size_col = foliation.get("size_col")
            position = np.array(foliation.get("placement_position"))

            if size_row and size_col:
                x_shift = position[0]
                y_shift = position[1]
                z_shift = position[2]

                for i in range(size_row):
                    for j in range(size_col):
                        obj_pose = create_pose_stamped_from_raw("base_link",
                                                                i * 0.1 - size_row * 0.1 / 2 + x_shift + 0.05,
                                                                j * 0.1 - size_col * 0.1 / 2 + y_shift + 0.05,
                                                                z_shift,
                                                                0, 0, 0, 1)

                        if collision_check(self.collision_manager, self.manipulated_object_mesh_path, obj_pose):
                            self.feasible_placements.append(convert_pose_stamped_to_matrix(obj_pose))
            else:
                print "no placement for foliation: " + foliation_name

    def _placement_circular(self, params):
        foliations = params["foliations"]
        for foliation in foliations:
            foliation_name = foliation["name"]
            reference_pose = np.array(foliation["reference_pose"])
            orientation_tolerance = foliation["orientation_tolerance"]
            position_tolerance = foliation["position_tolerance"]
            self._create_foliation(foliation_name, "base_link", reference_pose, orientation_tolerance,
                                   position_tolerance, self.feasible_grasps, self.sliding_similarity_matrix)

            center_position = np.array(foliation.get("placement_position"))
            radius = foliation.get("radius")
            start_angle = foliation.get("start_angle")
            end_angle = foliation.get("end_angle")
            steps = foliation.get("steps")

            if steps:
                angles = np.linspace(start_angle, end_angle, steps)

                for angle in angles:
                    x = center_position[0] + radius * np.cos(angle)
                    y = center_position[1] + radius * np.sin(angle)
                    z = center_position[2]
                    orientation = tf_trans.quaternion_from_euler(0, 0,
                                                        0 + angle)
                    obj_pose = create_pose_stamped_from_raw("base_link", x, y, z,
                                                            orientation[0], orientation[1], orientation[2], orientation[3])

                    if collision_check(self.collision_manager, self.manipulated_object_mesh_path, obj_pose):
                        self.feasible_placements.append(convert_pose_stamped_to_matrix(obj_pose))
            else:
                print "no placement for foliation: " + foliation_name

    def _placement_linear(self, params):
        foliations = params["foliations"]
        for foliation in foliations:
            foliation_name = foliation.get("name")
            reference_pose = np.array(foliation.get("reference_pose"))
            orientation_tolerance = foliation.get("orientation_tolerance")
            position_tolerance = foliation.get("position_tolerance")
            self._create_foliation(foliation_name, "base_link", reference_pose, orientation_tolerance,
                                   position_tolerance, self.feasible_grasps, self.sliding_similarity_matrix)

            start_position = np.array(foliation.get("start_position"))
            end_position = np.array(foliation.get("end_position"))
            num_steps = foliation.get("steps")

            if num_steps:
                positions_to_place = [start_position]
                for step in range(1, num_steps):
                    current_position = start_position + (end_position - start_position) * (float(step) / num_steps)
                    positions_to_place.append(current_position)
                positions_to_place.append(end_position)
                for position in positions_to_place:
                    obj_pose = create_pose_stamped_from_raw("base_link", position[0], position[1], position[2],
                                                            0, 0, 0, 1)

                    if collision_check(self.collision_manager, self.manipulated_object_mesh_path, obj_pose):
                        self.feasible_placements.append(convert_pose_stamped_to_matrix(obj_pose))
            else:
                print "no placement for foliation: " + foliation_name

    def _find_feasible_placements(self, params):
        placement_type = params["type"]
        if placement_type == "rectangular":
            self._placement_rectangular(params)
        elif placement_type == "linear":
            self._placement_linear(params)
        elif placement_type == "circular":
            self._placement_circular(params)
        else:
            raise Exception("Invalid placement type, check config")

    def _find_feasible_grasps(self, num_samples, rotated_matrix):
        loaded_array = np.load(self.grasp_poses_file)
        if num_samples == 0:
            num_samples = len(loaded_array.files)
        for ind in random.sample(list(range(len(loaded_array.files))), num_samples):
            self.feasible_grasps.append(np.dot(loaded_array[loaded_array.files[ind]], rotated_matrix))
        random.shuffle(self.feasible_grasps)

    def _calc_similarity_matrix(self, feasible_grasps):
        # calculate sliding similarity matrix
        different_matrix = np.zeros((len(feasible_grasps), len(feasible_grasps)))
        for i, grasp in enumerate(feasible_grasps):
            for j, grasp in enumerate(feasible_grasps):
                if i == j:
                    different_matrix[i, j] = 0
                different_matrix[i, j] = get_position_difference_between_poses(feasible_grasps[i],
                                                                               feasible_grasps[j])

        sliding_similarity_matrix = np.zeros((len(feasible_grasps), len(feasible_grasps)))
        max_distance = np.max(different_matrix)
        for i, grasp in enumerate(feasible_grasps):
            for j, grasp in enumerate(feasible_grasps):
                sliding_similarity_matrix[i, j] = gaussian_similarity(different_matrix[i, j], max_distance,
                                                                      sigma=self.similarity_sigma)

        self.sliding_similarity_matrix = sliding_similarity_matrix

    def _create_grasp_foliation(self, name='regrasp', frame_id="base_link"):
        foliation_grasp = ManipulationFoliation(foliation_name=name,
                                                constraint_parameters={
                                                    "frame_id": frame_id,
                                                    "is_object_in_hand": False,
                                                    "object_mesh_path": self.manipulated_object_mesh_path,
                                                    "obstacle_mesh": self.env_mesh_path,
                                                    "obstacle_pose": convert_pose_stamped_to_matrix(self.env_pose)
                                                },
                                                co_parameters=self.feasible_placements,
                                                similarity_matrix=np.identity(len(self.feasible_placements)))
        self.foliation_grasp = foliation_grasp

    def _create_foliation(self, name, frame_id, reference_pose, orientation_tolerance, position_tolerance, co_parameters, similarity_matrix):

        foliation_placement = ManipulationFoliation(foliation_name=name,
                                                    constraint_parameters={
                                                        "frame_id": frame_id,
                                                        "is_object_in_hand": True,
                                                        'object_mesh_path': self.manipulated_object_mesh_path,
                                                        "obstacle_mesh": self.env_mesh_path,
                                                        "obstacle_pose": convert_pose_stamped_to_matrix(self.env_pose),
                                                        "reference_pose": reference_pose,
                                                        "orientation_tolerance": orientation_tolerance,
                                                        "position_tolerance": position_tolerance
                                                    },
                                                    co_parameters=co_parameters,
                                                    similarity_matrix=similarity_matrix)
        self.foliation_placement_group.append(foliation_placement)


class RobotScene(object):
    def __init__(self, config):
        self.joint_state_dict = config.get('initial_joint_state')
        self.robot = None
        self.scene = None
        self.move_group = None
        self.compute_ik_srv = None

        self.initialize()

    def initialize(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('create_experiment_node', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(0.5)
        self.scene.clear()

        self.move_group = moveit_commander.MoveGroupCommander("arm")
        rospy.wait_for_service("/compute_ik")
        self.compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        self._set_initial_joint_state()

    def _set_initial_joint_state(self):
        joint_state_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = list(self.joint_state_dict.keys())
        joint_state.position = list(self.joint_state_dict.values())

        rate = rospy.Rate(10)
        while joint_state_publisher.get_num_connections() < 1:
            rate.sleep()
        joint_state_publisher.publish(joint_state)


class Sampler:
    def __init__(self, config, robot_scene):
        self.robot = robot_scene.robot
        self.move_group = robot_scene.move_group
        self.compute_ik_srv = robot_scene.compute_ik_srv
        self.manipulated_object_mesh_path = config.package_path + config.get('environment',
                                                                             'manipulated_object_mesh_path')
        self.env_pose = create_pose_stamped(config.get('environment', 'env_pose'))
        self.env_mesh_path = config.package_path + config.get('environment', 'env_mesh_path')
        self.scene = robot_scene.scene
        self.fraction = 0.97
        self.target_frame_id = "base_link"
        self.target_group_name = "arm"
        self.grasp_pose_mat = [[1, 0, 0, -0.05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    def _check_tolerance(self, reference_pose, position_tolerance, orientation_tolerance, position):
        ref_translation = reference_pose[:3, 3]
        pos_translation = position[:3, 3]
        translation_diff = np.abs(ref_translation - pos_translation)
        within_position_tolerance = np.all(translation_diff <= position_tolerance)

        ref_rotation_matrix = reference_pose[:3, :3]
        pos_rotation_matrix = position[:3, :3]
        relative_rotation_matrix = np.dot(ref_rotation_matrix.T, pos_rotation_matrix)

        relative_transform = np.identity(4)
        relative_transform[:3, :3] = relative_rotation_matrix

        relative_quaternion = tf_trans.quaternion_from_matrix(relative_transform)
        relative_euler_angles = tf_trans.euler_from_quaternion(relative_quaternion)
        within_orientation_tolerance = np.all(np.abs(relative_euler_angles) <= orientation_tolerance)

        # print relative_euler_angles, within_orientation_tolerance
        return within_position_tolerance and within_orientation_tolerance

    def sampling_func(self, foliation_1, foliation_2):
        co_parameters1 = foliation_1.co_parameters
        co_parameters2 = foliation_2.co_parameters

        # random choose param1 and param2
        selected_co_parameters1_index = random.randint(0, len(co_parameters1) - 1)

        # random set grasp (slide)
        grasp = co_parameters1[selected_co_parameters1_index]

        # random select placement and check if valid
        shuffled_indices = range(len(co_parameters2))
        random.shuffle(shuffled_indices)

        selected_co_parameters2_index = None
        placement = None
        found_valid_sample = False

        reference_pose = foliation_1.constraint_parameters.get("reference_pose")
        position_tolerance = foliation_1.constraint_parameters.get("position_tolerance")
        orientation_tolerance = foliation_1.constraint_parameters.get("orientation_tolerance")


        for index in shuffled_indices:
            placement = co_parameters2[index]
            tolerance = self._check_tolerance(reference_pose, position_tolerance, orientation_tolerance, placement)
            if tolerance:
                selected_co_parameters2_index = index
                found_valid_sample = True
                break

        if not found_valid_sample:
            print "Can't sample any valid co_parameter, skip..."
            return False, selected_co_parameters1_index, selected_co_parameters2_index, None

        # calc grasp pose based on base_link
        grasp_pose_mat = np.dot(placement, grasp)
        pre_grasp_pose_mat = np.dot(grasp_pose_mat, np.array(self.grasp_pose_mat))

        # set IK target pose
        ik_target_pose = PoseStamped()
        ik_target_pose.header.stamp = rospy.Time.now()
        ik_target_pose.header.frame_id = self.target_frame_id
        ik_target_pose.pose = msgify(geometry_msgs.msg.Pose, grasp_pose_mat)

        ik_req = GetPositionIKRequest()
        ik_req.ik_request.group_name = self.target_group_name
        ik_req.ik_request.avoid_collisions = True
        ik_req.ik_request.pose_stamped = ik_target_pose

        # random set robot pose
        random_moveit_robot_state = self.robot.get_current_state()
        random_position_list = list(random_moveit_robot_state.joint_state.position)
        for joint_name, joint_value in zip(self.move_group.get_joints(), self.move_group.get_random_joint_values()):
            random_position_list[random_moveit_robot_state.joint_state.name.index(joint_name)] = joint_value
        random_moveit_robot_state.joint_state.position = tuple(random_position_list)
        ik_req.ik_request.robot_state = random_moveit_robot_state

        ik_res = self.compute_ik_srv(ik_req)

        if not ik_res.error_code.val == MoveItErrorCodes.SUCCESS:
            return False, selected_co_parameters1_index, selected_co_parameters2_index, None

        # check motion
        moveit_robot_state = self.robot.get_current_state()
        moveit_robot_state.joint_state.position = ik_res.solution.joint_state.position

        self.move_group.set_start_state(moveit_robot_state)
        (planned_motion, fraction) = self.move_group.compute_cartesian_path(
            [msgify(geometry_msgs.msg.Pose, pre_grasp_pose_mat)], 0.01, 0.0)

        if fraction < self.fraction:
            return False, selected_co_parameters1_index, selected_co_parameters2_index, None

        intersection_motion = np.array([p.positions for p in planned_motion.joint_trajectory.points])

        return True, selected_co_parameters1_index, selected_co_parameters2_index, ManipulationIntersection(
            'release',
            intersection_motion,
            self.move_group.get_active_joints(),
            placement,
            self.manipulated_object_mesh_path,
            convert_pose_stamped_to_matrix(self.env_pose),
            self.env_mesh_path
        )

    def prepare_sampling_func(self):
        self.scene.add_mesh('env_obstacle', self.env_pose, self.env_mesh_path)

    def warp_sampling_func(self):
        self.scene.clear()


class ProblemVisualizer:
    def __init__(self, config, foliated_builder):
        self.env_mesh_path = "package://task_planner/" + config.get('environment', 'env_mesh_path')
        self.manipulated_object_mesh_path = "package://task_planner/" + config.get("environment",
                                                                                   "manipulated_object_mesh_path")
        self.env_pose = create_pose_stamped(config.get('environment', 'env_pose'))
        self.feasible_placements = foliated_builder.feasible_placements
        self.visualize_problem()

    def visualize_problem(self):
        problem_publisher = rospy.Publisher('/problem_visualization_marker_array', MarkerArray, queue_size=5)
        rospy.sleep(1)

        marker_array = MarkerArray()

        # visualize the obstacle
        obstacle_marker = self.create_marker(self.env_pose.pose, self.env_mesh_path, "obstacle", 0)
        marker_array.markers.append(obstacle_marker)

        # visualize feasible placements
        for i, placement in enumerate(self.feasible_placements):
            placement = msgify(geometry_msgs.msg.Pose, placement)
            object_marker = self.create_marker(placement, self.manipulated_object_mesh_path, "placement", i + 1)
            marker_array.markers.append(object_marker)

        problem_publisher.publish(marker_array)

    @staticmethod
    def create_marker(pose, mesh_path, namespace, marker_id):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale = Point(1, 1, 1)
        marker.color = ColorRGBA(0.5, 0.5, 0.5, 1)
        marker.mesh_resource = mesh_path
        return marker


if __name__ == "__main__":
    # load problem configuration
    config = Config("task_planner")

    # build environment and init state
    robot_scene = RobotScene(config)

    # build foliation
    foliated_builder = FoliatedBuilder(config)

    # visualize problem
    visualize_problem = ProblemVisualizer(config, foliated_builder)

    # build sampler
    sampler = Sampler(config, robot_scene)

    # sample pipeline
    foliated_intersections = []
    foliations = [foliated_builder.foliation_grasp] + foliated_builder.foliation_placement_group
    for placement_foliation in foliated_builder.foliation_placement_group:
        foliated_intersection = FoliatedIntersection(placement_foliation, foliated_builder.foliation_grasp,
                                                     sampler.sampling_func, sampler.prepare_sampling_func,
                                                     sampler.warp_sampling_func)
        foliated_intersections.append(foliated_intersection)

    # build problem
    foliated_problem = FoliatedProblem(config.get("task_parameters", 'task_name'))
    foliated_problem.set_foliation_n_foliated_intersection(foliations, foliated_intersections)
    foliated_problem.sample_intersections(config.get('task_parameters', 'num_samples'))

    # set the start and goal candidates
    start_candidates = []
    goal_candidates = []
    for p in range(len(foliated_builder.feasible_placements)):
        start_candidates.append((0, p))
        goal_candidates.append((0, p))

    foliated_problem.set_start_manifold_candidates(start_candidates)
    foliated_problem.set_goal_manifold_candidates(goal_candidates)

    # save problem
    foliated_problem.save(config.package_path + config.get('task_parameters', 'save_path'))
    loaded_foliated_problem = FoliatedProblem.load(ManipulationFoliation, ManipulationIntersection,
                                                   config.package_path + config.get('task_parameters', 'save_path'))

    # visualize problem
    visualize_problem = ProblemVisualizer(config, foliated_builder)

    # warp up
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
