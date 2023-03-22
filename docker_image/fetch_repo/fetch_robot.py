#!/usr/bin/env python3
import sys
sys.path.append('/root/zmqRemoteApi/clients/python')
from zmqRemoteApi import RemoteAPIClient

class FetchRobot:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.name = 'fetch'
        self.robot_object = self.sim.getObject('/'+self.name+'_respondable')

    def getJointNames(self):
        result = [self.sim.getObjectAlias(i) for i in self.sim.getObjectsInTree(self.robot_object, self.sim.object_joint_type)]
        return result


def main():
    fetch_robot = FetchRobot()
    print(fetch_robot.getJointNames())

if __name__ == "__main__":
    main()