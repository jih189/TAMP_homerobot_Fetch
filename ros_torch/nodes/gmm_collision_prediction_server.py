#!/usr/bin/env python3.8

# display the python version
import sys
print(sys.version)

import rospy
from ros_tensorflow_msgs.srv import GMMCollisionPredict, GMMCollisionPredictRequest, GMMCollisionPredictResponse
from gmm_collision_prediction.model import ModelWrapper
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ros_point_cloud_utils import pointcloud2_to_xyz_array
from scipy.spatial.transform import Rotation as R

import numpy as np

class RosInterface():
    def __init__(self):
        self.wrapped_model = ModelWrapper()
        self.predict_srv = rospy.Service('GMMCollisionPredict', GMMCollisionPredict, self.predict_cb)


    def predict_cb(self, req):
        rospy.loginfo("Prediction from service")
        pc_base_frame = pointcloud2_to_xyz_array(req.point_cloud)
        weights = self.wrapped_model.predict(pc_base_frame)

        resp = GMMCollisionPredictResponse()
        resp.weights = []
        for weight in weights:
            resp.weights.append(weight)
        return resp


def main():
    rospy.init_node("GMMCollisionPredict")
    rospy.loginfo("Creating the Torch Model")
    ri = RosInterface()
    rospy.loginfo("GMM Collision Prediction node initialized at service: GMMCollisionPredict")
    rospy.spin()

if __name__ == "__main__":
    main()
