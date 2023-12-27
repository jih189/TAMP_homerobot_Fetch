# -*- coding: utf-8 -*-

import rospy
import random
import numpy as np
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from utils import convert_matrix_to_pose_stamped, convert_pose_stamped_to_matrix

class ProblemSampler(object):
    def __init__(self, robot, move_group, compute_ik_srv):
        self.robot = robot
        self.move_group = move_group
        self.compute_ik_srv = compute_ik_srv

    def sample(self, co_parameters1, co_parameters2):
        selected_co_parameters1_index = random.randint(0, len(co_parameters1) - 1)
        selected_co_parameters2_index = random.randint(0, len(co_parameters2) - 1)
        placement = co_parameters2[selected_co_parameters2_index]
        grasp = co_parameters1[selected_co_parameters1_index]

        grasp_pose_matrix = np.dot(placement, grasp)
        grasp_pose_stamped = convert_matrix_to_pose_stamped(grasp_pose_matrix, "base_link")

        ik_req = GetPositionIKRequest()
        ik_req.ik_request.group_name = self.move_group.get_name()
        ik_req.ik_request.avoid_collisions = True
        ik_req.ik_request.pose_stamped = grasp_pose_stamped

        random_robot_state = self.robot.get_current_state()
        random_robot_state = self._set_random_robot_state(random_robot_state)
        ik_req.ik_request.robot_state = random_robot_state

        ik_res = self.compute_ik_srv(ik_req)
        if ik_res.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, selected_co_parameters1_index, selected_co_parameters2_index, None

        motion_plan = self._plan_motion(grasp_pose_matrix, random_robot_state)
        if motion_plan is None:
            return False, selected_co_parameters1_index, selected_co_parameters2_index, None

        return True, selected_co_parameters1_index, selected_co_parameters2_index, motion_plan

    def _set_random_robot_state(self, robot_state):
        joint_values = self.move_group.get_random_joint_values()
        for joint_name, joint_value in zip(self.move_group.get_active_joints(), joint_values):
            idx = robot_state.joint_state.name.index(joint_name)
            robot_state.joint_state.position[idx] = joint_value
        return robot_state

    def _plan_motion(self, grasp_pose_matrix, robot_state):

        return None

def convert_matrix_to_pose_stamped(matrix, frame_id):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.pose = convert_matrix_to_pose(matrix)
    return pose_stamped
