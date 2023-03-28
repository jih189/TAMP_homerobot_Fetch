#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import random
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class TrajectoryGenerator:
    def __init__(self, mc):
        self.robot = mc.RobotCommander()
        self.scene = mc.PlanningSceneInterface()
        self.move_group = mc.MoveGroupCommander("arm")
        self.state_validity_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        self.joint_names = self.move_group.get_active_joints()

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

def main():
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    trajectory_generator = TrajectoryGenerator(moveit_commander)
    sampled_trajectory = trajectory_generator.generateValidTrajectory()
    print sampled_trajectory


if __name__ == '__main__':
    main()
