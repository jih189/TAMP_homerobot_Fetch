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
from manipulation_test.srv import *
import random
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

if __name__ == "__main__":

    rospack = rospkg.RosPack()
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')

    # load the expierment
    experiment = Experiment()
    experiment.load(package_path + "/experiment_dir/pick_and_place")

    experiment.print_experiment_data()

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

    