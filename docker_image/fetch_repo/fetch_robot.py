#!/usr/bin/env python3
import sys
sys.path.append('/root/zmqRemoteApi/clients/python')
from zmqRemoteApi import RemoteAPIClient
from collision_detection_helper import COLLISION_IGNORE_LIST

# for debugs
import time

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

        # robot shape body handle list
        self.robot_body_handle = [h for h in self.sim.getObjectsInTree(self.robot_handle, self.sim.object_shape_type) if self.sim.getObjectAlias(h).endswith("_respondable")]

        # arm shape body handle list
        self.arm_body_handle = [h for h in self.sim.getObjectsInTree(self.arm_base_handle, self.sim.object_shape_type) if self.sim.getObjectAlias(h).endswith('_respondable')]

        self.collision_ignore_pair_list = [tuple(sorted([self.sim.getObject('/'+self.name+'_respondable' + '/' + b1 + '_respondable'), self.sim.getObject('/'+self.name+'_respondable' + '/' + b2 + '_respondable')])) for b1, b2 in COLLISION_IGNORE_LIST]

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
        
        for i in range(len(self.arm_joint_handles)):
            self.sim.setJointPosition(self.arm_joint_handles[i], float(values[i]))

        return True

    '''
    Get arm joint Velocity.
    '''
    def getArmJointVelocity(self):
        return [self.sim.getJointVelocity(h) for h in self.arm_joint_handles]

    '''
    Check self collision. In this work, we only check the collision on arm and finger for efficiency.
    '''
    def isSelfColliding(self):
        start_time = time.time()
        # print(self.collision_ignore_pair_list)
        for a_h in self.arm_body_handle:
            for r_h in self.robot_body_handle:
                if a_h == r_h:
                    continue
                if tuple(sorted([r_h, a_h])) in self.collision_ignore_pair_list:
                    continue
                c1 = time.time()
                result, collidingHandle = self.sim.checkCollision(a_h, r_h)
                c2 = time.time()
                print("collision time ", c2 - c1)
                if result:
                    print("Collision happend between ", self.sim.getObjectAlias(a_h)[:-12], " and ", self.sim.getObjectAlias(r_h)[:-12])
        end_time = time.time()
        print("Elasped time: ", end_time - start_time)
        # check collision on arm.
        # get body

        return False


def main():
    fetch_robot = FetchRobot()
    fetch_robot.isSelfColliding()

if __name__ == "__main__":
    main()