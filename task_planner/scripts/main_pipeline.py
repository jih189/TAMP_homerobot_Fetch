#!/usr/bin/env python
from experiment_helper import Experiment, Manifold, Intersection
from jiaming_task_planner import MTGTaskPlanner

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
from manipulation_test.srv import *
import random
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

if __name__ == "__main__":

    max_attempt_times = 1

    rospack = rospkg.RosPack()
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')

    # load the expierment
    experiment = Experiment()
    experiment.load(package_path + "/experiment_dir/pick_and_place")

    # load the experiment into the task planner
    task_planner = MTGTaskPlanner()

    task_planner.reset_task_planner()

    # setup the task graph.
    # add manifolds
    for manifold in experiment.manifolds:
        # the manifold id in the task planner should be a pair of (foliation_id, manifold_id)
        # TODO: we need to add the manifold constraint instead of None
        task_planner.add_manifold(None, (manifold.foliation_id, manifold.manifold_id))

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

    # robot initial configuration [-1.28, 1.52, 0.35, 1.81, 0.0, 1.47, 0.0]

    # set start and goal configurations
    task_planner.set_start_and_goal(
        (0,0), # start manifold id
        (
            False, 
            [[-1.28, 1.52, 0.35, 1.81, 0.0, 1.47, 0.0]], 
            None,
            None,
            None
        ), # start configuration
        (2,0), # goal manifold id
        (
            False, 
            [[-1.28, 1.52, 0.35, 1.81, 0.0, 1.47, 0.0]], 
            None,
            None,
            None
        ) # goal configuration
    )

    for _ in range(max_attempt_times):
        # generate task sequence
        task_sequence = task_planner.generate_task_sequence()
        found_solution = True
        soulution_path = []

        # motion planner tries to solve each task in the task sequence
        for task in task_sequence:
            # print the task detail here
            print("----------- task:")
            print("start configuration:")
            print(task.start_configuration[1][-1])
            print("goal configuration:")
            print(task.goal_configuration[1][0])
        break

    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('main_pipeline_node', anonymous=True)

    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # scene.clear()
    # move_group = moveit_commander.MoveGroupCommander("arm")
    # rospy.wait_for_service("/compute_ik")
    # compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

    # # set initial joint state
    # joint_state_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)

    # # Create a JointState message
    # joint_state = JointState()
    # joint_state.header.stamp = rospy.Time.now()
    # joint_state.name = ['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'wrist_flex_joint', 'l_gripper_finger_joint', 'r_gripper_finger_joint']
    # joint_state.position = [0.38, -1.28, 1.52, 0.35, 1.81, 1.47, 0.04, 0.04]

    # rate = rospy.Rate(10)
    # while(joint_state_publisher.get_num_connections() < 1): # need to wait until the publisher is ready.
    #     rate.sleep()
    # joint_state_publisher.publish(joint_state)

    # print("init joint position")
    # print(move_group.get_current_joint_values())