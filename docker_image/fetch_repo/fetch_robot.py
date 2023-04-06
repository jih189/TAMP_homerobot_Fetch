#!/usr/bin/env python3
import sys
sys.path.append('/root/zmqRemoteApi/clients/python')
from zmqRemoteApi import RemoteAPIClient
from collision_detection_helper import COLLISION_IGNORE_LIST

# for debugs
import time

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class FetchRobot:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.name = 'fetch'
        self.robot_handle = self.sim.getObject('/'+self.name+'_respondable')
        self.arm_base_handle = self.sim.getObject('/'+self.name+'_respondable/shoulder_pan_link_respondable')
        self.arm_joint_names = ['shoulder_pan_joint', \
                                'shoulder_lift_joint', \
                                'upperarm_roll_joint', \
                                'elbow_flex_joint', \
                                'forearm_roll_joint', \
                                'wrist_flex_joint', \
                                'wrist_roll_joint']
        self.arm_joint_handles = [self.sim.getObject('/'+self.name+'_respondable' + '/' + n) for n in self.arm_joint_names]
        self.torso_joint_name = 'torso_lift_joint'
        self.torso_joint_handle = self.sim.getObject('/'+self.name+'_respondable' + '/' + self.torso_joint_name)
        self.finger_joint_names = ['l_gripper_finger_joint', 'r_gripper_finger_joint']
        self.finger_joint_handles = [self.sim.getObject('/'+self.name+'_respondable' + '/' + n) for n in self.finger_joint_names]

        self.initialPosition = self.getPosition()
        self.initialArmPosition = [-1.398378610610962, 1.3106462955474854, 0.5251686573028564, 1.6612961292266846, -0.002762574702501297, 1.4766314029693604, 1.5701130628585815]
        # we use ros controller to controll the arm for ROS env
        self.arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    '''
    Get the arm joint names.
    '''
    def getArmJointNames(self):
        return self.arm_joint_names

    '''
    Get the torso_joint_name.
    '''
    def getTorsoJointName(self):
        return self.torso_joint_name

    '''
    Get fingers' names.
    '''
    def getFingerJointNames(self):
        return self.finger_joint_names

    '''
    Get arm joint position.
    '''
    def getArmJointPosition(self):
        return [self.sim.getJointPosition(h) for h in self.arm_joint_handles]

    '''
    Set arm joint Position.
    '''
    def setArmJointPosition(self, values):
        if len(values) != len(self.arm_joint_handles):
            print("the input's length is not equal to the arm joint number.")
            return False
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = values
        point.time_from_start = rospy.Duration(0.1)
        goal.trajectory.points.append(point)
        self.arm_client.send_goal(goal)

        for i in range(len(self.arm_joint_handles)):
            self.sim.setJointPosition(self.arm_joint_handles[i], float(values[i]))

    '''
    Get arm joint Velocity.
    '''
    def getArmJointVelocity(self):
        return [self.sim.getJointVelocity(h) for h in self.arm_joint_handles]

    '''
    Get Robot's postion
    '''
    def getPosition(self):
        return self.sim.getObjectPosition(self.robot_handle, self.sim.handle_world)

    '''
    Set Robot's position
    '''
    def setPosition(self, position):
        self.sim.setObjectPosition(self.robot_handle, self.sim.handle_world, position)

    '''
    Reset initial position
    '''
    def resetPosition(self):
        self.setPosition(self.initialPosition)

    '''
    Reset arm position
    '''
    def resetArmPosition(self):
        self.setArmJointPosition(self.initialArmPosition)

    '''
    Reset environment
    '''
    def resetScene(self):
        self.sim.pauseSimulation()

        # reset arm position
        self.resetArmPosition()

        # reset position
        self.resetPosition()

        # time.sleep(1)

        self.sim.startSimulation()

def main():
    fetch_robot = FetchRobot()

if __name__ == "__main__":
    main()