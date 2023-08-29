#!/usr/bin/env python
import rospy
from ros_tensorflow_msgs.srv import GMMCollisionPredict, GMMCollisionPredictRequest
import numpy as np
from sensor_msgs.msg import PointCloud2
import ros_numpy

def array_to_pointcloud2(pc):

    """
    pc: 2d numpy array of shape (N, 4)
    """

    pc_array = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
    ])
    pc_array['x'] = pc[:, 0]
    pc_array['y'] = pc[:, 1]
    pc_array['z'] = pc[:, 2]
    pc_msg = ros_numpy.msgify(PointCloud2, pc_array)
    return pc_msg

class RosInterface():
    def __init__(self):
        self.predict_client = rospy.ServiceProxy('GMMCollisionPredict', GMMCollisionPredict)

    def call_client(self, pc):
        request = GMMCollisionPredictRequest(array_to_pointcloud2(pc))
        res = self.predict_client(request)
        return np.array(res.weights)
