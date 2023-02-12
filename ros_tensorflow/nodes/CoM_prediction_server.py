#!/usr/bin/env python3

# display the python version
import sys
print(sys.version)

import rospy
from ros_tensorflow_msgs.srv import ComPredict, ComPredictRequest, ComPredictResponse
from CoM_prediction.model import ModelWrapper, BATCH_SIZE, NUM_POINT
from geometry_msgs.msg import Point

import numpy as np
import ros_numpy

class RosInterface():
    def __init__(self):
        self.input_dim = 10
        self.output_dim = 2

        # This dictionary is only used for synthetic data generation
        self.fake_hidden_params = {}

        self.wrapped_model = ModelWrapper()

        self.predict_srv = rospy.Service('CoMPredict', ComPredict, self.predict_cb)


    def predict_cb(self, req):
        rospy.loginfo("Prediction from service")
        table_point_cloud_in_world = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(req.table_point_cloud)
        segmented_point_cloud_in_world = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(req.segmented_point_cloud)
        # concate the two point clouds and create a 2d label array
        labels = np.zeros((table_point_cloud_in_world.shape[0] + segmented_point_cloud_in_world.shape[0], 1))
        labels[:table_point_cloud_in_world.shape[0]] = 1
        point_cloud = np.concatenate((table_point_cloud_in_world, segmented_point_cloud_in_world), axis=0)
        # concate the point cloud with the label array
        point_cloud_labels = np.concatenate((point_cloud, labels), axis=1)
        # shuffle the point cloud and label array
        np.random.shuffle(point_cloud_labels)
        # reshape the point cloud and label array to (BATCH_SIZE, NUM_POINT, 4)
        point_cloud_labels = point_cloud_labels.reshape((BATCH_SIZE, NUM_POINT, 4))
        CoM = self.wrapped_model.predict(point_cloud_labels)
        # convert the CoM to a Point message
        CoM_msg = Point()
        CoM_msg.x = CoM[0]
        CoM_msg.y = CoM[1]
        CoM_msg.z = CoM[2]
        return ComPredictResponse(com = CoM_msg)

def main():
    rospy.init_node("ros_tensorflow")
    rospy.loginfo("Creating the Tensorflow model")
    ri = RosInterface()
    rospy.loginfo("CoMPredict node initialized at service: CoMPredict")
    rospy.spin()

if __name__ == "__main__":
    main()
