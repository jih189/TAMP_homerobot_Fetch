#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import random
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np
import pickle
from os import path as osp
import os
import shutil

class TrajectoryGenerator:
    def __init__(self, mc):
        self.robot = mc.RobotCommander()
        self.scene = mc.PlanningSceneInterface()
        self.move_group = mc.MoveGroupCommander("arm")
        self.state_validity_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        self.joint_names = self.move_group.get_active_joints()

        # # set initial joint state
        joint_state_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)

        # Create a JointState message
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'wrist_flex_joint']
        joint_state.position = [-1.28, 1.52, 0.35, 1.81, 1.47]

        rate = rospy.Rate(10)
        while(joint_state_publisher.get_num_connections() < 1): # need to wait until the publisher is ready.
            rate.sleep()
        joint_state_publisher.publish(joint_state)

        # set the planner to rrt star
        self.move_group.set_planner_id('RRTstarkConfigDefault')
        self.move_group.set_planning_time(1.0)

    def getValidJoints(self):
        '''
        Return a valid joint values of the Fetch. If something stuck here, it can be
        caused by too little joint values are valid.
        '''
        while True:
            joint_values = self.move_group.get_random_joint_values()
            # Create a GetStateValidityRequest object
            request = GetStateValidityRequest()
            request.robot_state.joint_state.name = self.joint_names
            request.robot_state.joint_state.position = joint_values
            if self.state_validity_service(request).valid:
                return joint_values

    def generateValidTrajectory(self):
        '''
        It first samples two valid joint values, then plan for the trajectory between them.
        '''
        while True:
            start_joint = self.getValidJoints()
            target_joint = self.getValidJoints()

            moveit_robot_state = RobotState()
            moveit_robot_state.joint_state.name = self.joint_names
            moveit_robot_state.joint_state.position = start_joint

            self.move_group.set_start_state(moveit_robot_state)
            self.move_group.set_joint_value_target(target_joint)
            result = self.move_group.plan()
            if result[0]:
                sampled_trajectory = [j.positions for j in result[1].joint_trajectory.points]
                return sampled_trajectory

    def visualizeTrajectory(self, trajectory_data):
        robot_trajectory = RobotTrajectory()
        robot_trajectory.joint_trajectory.joint_names = self.joint_names
        currenttime = rospy.Duration(0.0)

        for point in trajectory_data.tolist():
            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.positions = point
            joint_trajectory_point.time_from_start = currenttime
            robot_trajectory.joint_trajectory.points.append(joint_trajectory_point)
            currenttime += rospy.Duration(0.5)

        # Create a DisplayTrajectory message
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory.append(robot_trajectory)

        display_trajectory_publisher = rospy.Publisher('/move_group/planned_path', DisplayTrajectory, queue_size=1)
        display_trajectory_publisher.publish(display_trajectory)



def main():
    rospy.init_node('data_trajectory_generation')

    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    trajectory_generator = TrajectoryGenerator(moveit_commander)
    
    # fileDir = 'trajectory_data/'

    # # remove the directory for data if it exists.
    # if os.path.exists(fileDir):
    #     shutil.rmtree(fileDir)

    # os.mkdir(fileDir)

    # for env_num in range(1):
    #     os.mkdir(fileDir + "env_%06d/" % env_num)
    #     for i in range(10):
    #         sampled_trajectory = np.array(trajectory_generator.generateValidTrajectory())
    #         trajData = {'path': sampled_trajectory}
    #         with open(fileDir + "env_%06d/" % env_num + "path_%d.p" % i, 'wb') as f:
    #             pickle.dump(trajData, f)

        # # Load the saved numpy array using pickle
        # with open(fileDir + "path_%d.p" % i, 'rb') as f:
        #     loaded_array = pickle.load(f)
        # print loaded_array['path']




if __name__ == '__main__':
    main()
