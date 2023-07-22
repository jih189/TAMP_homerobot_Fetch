#!/usr/bin/env python
from experiment_helper import Experiment, Manifold, Intersection
from jiaming_task_planner import MTGTaskPlanner, MDPTaskPlanner

import sys
import copy
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetJointWithConstraints, GetJointWithConstraintsRequest
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, MoveItErrorCodes
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import tf.transformations as tf_trans
from ros_numpy import numpify, msgify
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Point32
import trimesh
from trimesh import transformations
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
import struct
from manipulation_test.srv import *
import random
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

# convert a list of joint values to robotTrajectory
def convert_joint_values_to_robot_trajectory(joint_values_list_, joint_names_):
    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory = JointTrajectory()
    robot_trajectory.joint_trajectory.joint_names = joint_names_

    for i in range(len(joint_values_list_)):
        robot_trajectory.joint_trajectory.points.append(JointTrajectoryPoint())
        robot_trajectory.joint_trajectory.points[i].positions = joint_values_list_[i]
        robot_trajectory.joint_trajectory.points[i].velocities = [0.0] * len(joint_values_list_[i])
        robot_trajectory.joint_trajectory.points[i].accelerations = [0.0] * len(joint_values_list_[i])
        robot_trajectory.joint_trajectory.points[i].time_from_start = rospy.Duration(0.1 * i)

    return robot_trajectory

if __name__ == "__main__":

    max_attempt_times = 1

    rospack = rospkg.RosPack()
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')

    # load the expierment
    experiment = Experiment()
    experiment.load(package_path + "/experiment_dir/pick_and_place")

    # load the experiment into the task planner
    # task_planner = MTGTaskPlanner()
    task_planner = MDPTaskPlanner()

    task_planner.reset_task_planner()

    # setup the task graph.
    # add manifolds
    for manifold in experiment.manifolds:
        # the manifold id in the task planner should be a pair of (foliation_id, manifold_id)
        # the constraint here has (moveit constraint, has_object_in_hand, object_pose, object mesh, object name)
        # if the has_object_in_hand is true, then object pose here is the in-hand-object-pose which is the grasp pose in the object frame
        # if the has_object_in_hand is false, then object pose here is the object placement pose which is the placement pose in the world frame
        moveit_constraint = Constraints()
        manifold_object_pose = manifold.in_hand_pose if manifold.has_object_in_hand else manifold.object_pose
        # manifold.has_object_in_hand
        task_planner.add_manifold(
            (moveit_constraint, manifold.has_object_in_hand, manifold_object_pose, manifold.object_mesh, manifold.object_name), 
            (manifold.foliation_id, manifold.manifold_id)
        )

    # add intersections
    for intersection in experiment.intersections:
        task_planner.add_intersection(
            (intersection.foliation_id_1, intersection.manifold_id_1),
            (intersection.foliation_id_2, intersection.manifold_id_2),
            (
                intersection.has_object_in_hand, 
                intersection.trajectory_motion, 
                intersection.in_hand_pose,
                intersection.object_mesh,
                intersection.object_name
            )
        )

    # initialize the motion planner
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('main_pipeline_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    scene.clear()
    move_group = moveit_commander.MoveGroupCommander("arm")
    # move_group.set_planner_id('CBIRRTConfigDefault')
    rospy.wait_for_service("/compute_ik")
    compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

    display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,
        )

    # set initial joint state
    joint_state_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)

    # Create a JointState message
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'wrist_flex_joint', 'l_gripper_finger_joint', 'r_gripper_finger_joint']
    joint_state.position = [0.38, -1.28, 1.51, 0.35, 1.81, 1.47, 0.04, 0.04]

    rate = rospy.Rate(10)
    while(joint_state_publisher.get_num_connections() < 1): # need to wait until the publisher is ready.
        rate.sleep()
    joint_state_publisher.publish(joint_state)

    # load the obstacle into the planning scene.
    obstacle_pose_stamped = PoseStamped()
    obstacle_pose_stamped.header.frame_id = "base_link"
    obstacle_pose_stamped.pose = msgify(geometry_msgs.msg.Pose, experiment.obstacle_mesh_pose)
    
    print("Add the obstacle to the planning scene")
    scene.add_mesh("obstacle", obstacle_pose_stamped, experiment.obstacle_mesh, size=(1,1,1))

    # set start and goal configurations
    task_planner.set_start_and_goal(
        (0,0), # start manifold id
        (
            False, 
            [move_group.get_current_joint_values()], 
            None,
            None,
            None
        ), # start configuration
        (2,0), # goal manifold id
        (
            False, 
            [move_group.get_current_joint_values()], 
            None,
            None,
            None
        ) # goal configuration
    )

    for _ in range(max_attempt_times):
        # generate task sequence
        task_sequence = task_planner.generate_task_sequence()
        if len(task_sequence) == 0:
            print("no task sequence found")
            break

        found_solution = True
        solution_path = []

        # motion planner tries to solve each task in the task sequence
        for task in task_sequence:
            # print the task detail here
            # print("----------- task:")
            # print("start configuration:")
            # print(task.start_configuration[1][-1])
            # print("goal configuration:")
            # print(task.goal_configuration[1][0])
            # print("constraint:")
            # print("moveit constraint:")
            # print(task.constraint[0])
            # print("has_object_in_hand:")
            # print(task.constraint[1])
            # print("object_pose:")
            # print(task.constraint[2])
            # print("object_mesh:")
            # print(task.constraint[3])
            # print("object_name:")
            # print(task.constraint[4])
            # print("task graph info:")
            # print(task.task_graph_info)
            # print("-----------")

            if task.constraint[1]: # has object in hand
                
                # attach the object to the hand in the planning scene
                target_object_pose = PoseStamped()
                target_object_pose.header.frame_id = "base_link"
                target_object_pose.pose = msgify(geometry_msgs.msg.Pose, numpify(move_group.get_current_pose().pose).dot(np.linalg.inv(task.constraint[2])))
                scene.attach_mesh(
                    "wrist_roll_link", # link
                    task.constraint[4], # name
                    target_object_pose, # pose
                    task.constraint[3], # filename
                    size=(1,1,1), #size
                    touch_links=["l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"] #touch_links
                )

                # check whether the attached object is in the planning scene
                # if it is not, wait for short time.
                while task.constraint[4] not in scene.get_attached_objects():
                    rospy.sleep(0.0001)
                
                # do the motion planning
                move_group.clear_path_constraints()
                
                # set start and goal congfiguration to motion planner.
                moveit_robot_state = robot.get_current_state()
                start_position_list = list(moveit_robot_state.joint_state.position)
                for joint_name, joint_value in zip(move_group.get_active_joints(), task.start_configuration[1][-1]):
                    start_position_list[moveit_robot_state.joint_state.name.index(joint_name)] = joint_value
                moveit_robot_state.joint_state.position = tuple(start_position_list)
                move_group.set_start_state(moveit_robot_state)
                move_group.set_joint_value_target(task.goal_configuration[1][0])

                motion_plan_result = move_group.plan()

                task_planner.update(task.task_graph_info, motion_plan_result)

                if not motion_plan_result[0]: #if the motion planner can't find a solution, then replan
                    found_solution = False
                    break

                solution_path.append(motion_plan_result[1])

                # add the intersection motion to the solution path
                if(len(task.goal_configuration[1]) > 1):
                    intersection_motion = convert_joint_values_to_robot_trajectory(task.goal_configuration[1], move_group.get_active_joints())
                    solution_path.append(intersection_motion)

                # remove the attached object from the planning scene
                scene.remove_attached_object("wrist_roll_link", task.constraint[4])

                # check whether the attached object is in the planning scene
                # if it is, wait for short time.
                while task.constraint[4] in scene.get_attached_objects():
                    rospy.sleep(0.0001)

            else:
                # add the object to the planning scene
                target_object_pose = PoseStamped()
                target_object_pose.header.frame_id = "base_link"
                target_object_pose.pose = msgify(geometry_msgs.msg.Pose, task.constraint[2])
                scene.add_mesh(task.constraint[4], target_object_pose, task.constraint[3], size=(1,1,1))
    
                # check whether the object is in the planning scene
                # if it is not, wait for short time.
                while task.constraint[4] not in scene.get_known_object_names():
                    rospy.sleep(0.0001)

                # do the motion planning
                move_group.clear_path_constraints()
                
                # set start and goal congfiguration to motion planner.
                moveit_robot_state = robot.get_current_state()
                start_position_list = list(moveit_robot_state.joint_state.position)
                for joint_name, joint_value in zip(move_group.get_active_joints(), task.start_configuration[1][-1]):
                    start_position_list[moveit_robot_state.joint_state.name.index(joint_name)] = joint_value
                moveit_robot_state.joint_state.position = tuple(start_position_list)
                move_group.set_start_state(moveit_robot_state)
                move_group.set_joint_value_target(task.goal_configuration[1][0])

                motion_plan_result = move_group.plan()

                task_planner.update(task.task_graph_info, motion_plan_result)

                if not motion_plan_result[0]: #if the motion planner can't find a solution, then replan
                    found_solution = False
                    break

                solution_path.append(motion_plan_result[1])

                # add the intersection motion to the solution path
                if(len(task.goal_configuration[1]) > 1):
                    intersection_motion = convert_joint_values_to_robot_trajectory(task.goal_configuration[1], move_group.get_active_joints())
                    solution_path.append(intersection_motion)

                # remove the object from the planning scene
                scene.remove_world_object(task.constraint[4])

                # check whether the object is in the planning scene
                # if it is, wait for short time.
                while task.constraint[4] in scene.get_known_object_names():
                    rospy.sleep(0.0001)

        if found_solution: # found solution, then break the


            print("found solution")
            # try to execute the solution path

            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = move_group.get_current_state()
            display_trajectory.trajectory = solution_path

            display_trajectory_publisher.publish(display_trajectory)
            
            break