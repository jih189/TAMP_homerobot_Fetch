#!/usr/bin/env python
from experiment_helper import Experiment, Manifold, Intersection

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

if __name__ == "__main__":
    rospack = rospkg.RosPack()

    # Get the path of the desired package
    package_path = rospack.get_path("task_planner")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("create_experiment_node", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    # remove all objects from the scene, you need to give some time to the scene to update.
    rospy.sleep(0.5)
    scene.clear()

    move_group = moveit_commander.MoveGroupCommander("arm")
    rospy.wait_for_service("/compute_ik")
    compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

    # set initial joint state
    joint_state_publisher = rospy.Publisher(
        "/move_group/fake_controller_joint_states", JointState, queue_size=1
    )

    # Create a JointState message
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = [
        "torso_lift_joint",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "wrist_flex_joint",
        "l_gripper_finger_joint",
        "r_gripper_finger_joint",
    ]
    joint_state.position = [0.38, -1.28, 1.52, 0.35, 1.81, 1.47, 0.04, 0.04]

    rate = rospy.Rate(10)
    while (
        joint_state_publisher.get_num_connections() < 1
    ):  # need to wait until the publisher is ready.
        rate.sleep()
    joint_state_publisher.publish(joint_state)

    ######################################### start to generate the experiment #########################################

    experiment = Experiment()

    # load the obstacle
    env_pose = PoseStamped()
    env_pose.header.frame_id = "base_link"
    env_pose.pose.position.x = 0.51
    env_pose.pose.position.y = 0.05
    env_pose.pose.position.z = -0.02
    env_pose.pose.orientation.x = 0
    env_pose.pose.orientation.y = 0
    env_pose.pose.orientation.z = 0.707
    env_pose.pose.orientation.w = 0.707
    print("Add the maze desk to the planning scene")

    scene.add_mesh("maze", env_pose, package_path + "/mesh_dir/maze.stl")

    experiment.setup(
        "maze",
        package_path + "/mesh_dir/cup.stl",
        package_path + "/mesh_dir/maze.stl",
        numpify(env_pose.pose),
        robot.get_current_state().joint_state.position,
        robot.get_current_state().joint_state.name,
        move_group.get_joints(),
    )

    # load the object into trimesh
    env_mesh = trimesh.load_mesh(package_path + "/mesh_dir/maze.stl")
    env_pose_matrix = transformations.quaternion_matrix(
        [
            env_pose.pose.orientation.w,
            env_pose.pose.orientation.x,
            env_pose.pose.orientation.y,
            env_pose.pose.orientation.z,
        ]
    )
    env_pose_matrix[0, 3] = env_pose.pose.position.x
    env_pose_matrix[1, 3] = env_pose.pose.position.y
    env_pose_matrix[2, 3] = env_pose.pose.position.z
    env_mesh.apply_transform(env_pose_matrix)

    collision_manager = trimesh.collision.CollisionManager()
    collision_manager.add_object("env", env_mesh)

    num_of_row = 6
    num_of_col = 8
    x_shift = 0.56
    y_shift = 0.1
    z_shift = 0.8

    placement_pose_manifolds = []

    # for each feasible placement pose, add a manifold
    for i in range(num_of_row):
        for j in range(num_of_col):
            obj_mesh = trimesh.load_mesh(package_path + "/mesh_dir/cup.stl")

            # generate a random pose
            obj_pose = PoseStamped()
            obj_pose.header.frame_id = "base_link"
            obj_pose.pose.position.x = i * 0.1 - num_of_row * 0.1 / 2 + x_shift
            obj_pose.pose.position.y = j * 0.1 - num_of_col * 0.1 / 2 + y_shift
            obj_pose.pose.position.z = z_shift
            obj_pose.pose.orientation.x = 0
            obj_pose.pose.orientation.y = 0
            obj_pose.pose.orientation.z = 0
            obj_pose.pose.orientation.w = 1

            obj_pose_matrix = transformations.quaternion_matrix(
                [
                    obj_pose.pose.orientation.w,
                    obj_pose.pose.orientation.x,
                    obj_pose.pose.orientation.y,
                    obj_pose.pose.orientation.z,
                ]
            )
            obj_pose_matrix[0, 3] = obj_pose.pose.position.x
            obj_pose_matrix[1, 3] = obj_pose.pose.position.y
            obj_pose_matrix[2, 3] = obj_pose.pose.position.z
            obj_mesh.apply_transform(obj_pose_matrix)

            collision_manager.add_object("obj" + str(i * num_of_col + j), obj_mesh)

            if not collision_manager.in_collision_internal():
                # scene.add_mesh('cup' + str(i * num_of_col + j), obj_pose, package_path + '/mesh_dir/cup.stl')

                # add manifold with current placement pose as parameter
                current_manifold = Manifold(
                    0,  # placement foliation id
                    len(placement_pose_manifolds),  # placement manifold id
                    "cup",  # object name
                    package_path + "/mesh_dir/cup.stl",  # object mesh file name
                    False,  # is the object in hand
                )

                current_manifold.add_object_placement(obj_pose_matrix)

                placement_pose_manifolds.append(current_manifold)

                experiment.add_manifold(current_manifold)

            collision_manager.remove_object("obj" + str(i * num_of_col + j))

    # load all grasp poses over the object.
    grasp_pose_list = []

    loaded_array = np.load(package_path + "/mesh_dir/cup.npz")

    rotated_matrix = np.array(
        [[1, 0, 0, -0.17], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    )

    for ind in random.sample(list(range(len(loaded_array.files))), 20):
        grasp_pose_list.append(
            np.dot(loaded_array[loaded_array.files[ind]], rotated_matrix)
        )

    grasp_pose_manifolds = []

    table_top_pose = np.array(
        [[1, 0, 0, 0.5], [0, 1, 0, 0], [0, 0, 1, 0.8], [0, 0, 0, 1]]
    )

    # add each grasp pose as a manifold
    for g in grasp_pose_list:
        # create the manifold for this grasp
        grasp_manifold = Manifold(
            1,  # foliation id
            len(grasp_pose_manifolds),  # manifold id
            "cup",  # object name
            package_path + "/mesh_dir/cup.stl",  # object mesh file name
            True,  # is the object in hand
        )

        grasp_manifold.add_constraint(
            g,  # grasp pose in the object frame
            table_top_pose,  # constraint pose
            np.array([0.1, 0.1, 3.14 * 2]),  # orientation constraint
            np.array([2000, 2000, 0.05]),  # position constraint
        )

        grasp_pose_manifolds.append(grasp_manifold)

        experiment.add_manifold(grasp_manifold)

    # need to find all the intersections between placement and grasp manifolds,
    # and add them to the experiment
    for placement_manifold in placement_pose_manifolds:
        for grasp_manifold in grasp_pose_manifolds:
            # calculate the grasp pose in the world frame
            grasp_pose_mat = np.dot(
                placement_manifold.object_pose, grasp_manifold.in_hand_pose
            )
            pre_grasp_pose_mat = np.dot(
                grasp_pose_mat,
                np.array([[1, 0, 0, -0.09], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
            )

            # set the ik target pose
            ik_target_pose = PoseStamped()
            ik_target_pose.header.stamp = rospy.Time.now()
            ik_target_pose.header.frame_id = "base_link"
            ik_target_pose.pose = msgify(geometry_msgs.msg.Pose, grasp_pose_mat)

            ik_req = GetPositionIKRequest()
            ik_req.ik_request.group_name = "arm"
            ik_req.ik_request.avoid_collisions = True
            ik_req.ik_request.pose_stamped = ik_target_pose

            possible_ik_solutions = []

            for _ in range(10):
                # set the robot state randomly
                random_moveit_robot_state = robot.get_current_state()
                random_position_list = list(
                    random_moveit_robot_state.joint_state.position
                )
                for joint_name, joint_value in zip(
                    move_group.get_joints(), move_group.get_random_joint_values()
                ):
                    random_position_list[
                        random_moveit_robot_state.joint_state.name.index(joint_name)
                    ] = joint_value
                random_moveit_robot_state.joint_state.position = tuple(
                    random_position_list
                )
                ik_req.ik_request.robot_state = random_moveit_robot_state

                ik_res = compute_ik_srv(ik_req)

                if not ik_res.error_code.val == 1:
                    continue

                # if there is a similar solution in possible ik solution, then skip it
                if len(possible_ik_solutions) > 0:
                    if (
                        np.linalg.norm(
                            np.array(ik_res.solution.joint_state.position)
                            - np.array(possible_ik_solutions),
                            axis=1,
                        ).min()
                        < 0.01
                    ):
                        continue

                possible_ik_solutions.append(ik_res.solution.joint_state.position)

                # need to check the motion for pre-grasp
                moveit_robot_state = robot.get_current_state()
                moveit_robot_state.joint_state.position = (
                    ik_res.solution.joint_state.position
                )

                move_group.set_start_state(moveit_robot_state)
                (approach_plan, fraction) = move_group.compute_cartesian_path(
                    [msgify(geometry_msgs.msg.Pose, pre_grasp_pose_mat)], 0.01, 0.0
                )

                if fraction < 0.97:
                    continue

                # save the intersection motion from pre-grasp manifold to grasp manifold.
                intersection_motion = np.array(
                    [p.positions for p in approach_plan.joint_trajectory.points]
                )
                intersection = Intersection(
                    grasp_manifold.foliation_id,
                    grasp_manifold.manifold_id,
                    placement_manifold.foliation_id,
                    placement_manifold.manifold_id,
                    False,
                    intersection_motion,
                    placement_manifold.object_pose,
                    package_path + "/mesh_dir/cup.stl",
                    "cup",
                )
                experiment.add_intersection(intersection)

    # need to set start and goal foliation manifold id(You need to modify here.)
    experiment.set_start_and_goal_foliation_manifold_id(0, 0, 0, 4)

    experiment.save(package_path + "/experiment_dir/" + experiment.experiment_name)

    # shutdown the moveit
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
