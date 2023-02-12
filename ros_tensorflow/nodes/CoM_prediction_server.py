#!/usr/bin/env python3

# display the python version
import sys
print(sys.version)

import rospy
from ros_tensorflow_msgs.srv import ComPredict, ComPredictRequest, ComPredictResponse
from CoM_prediction.model import ModelWrapper, BATCH_SIZE, NUM_POINT
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R

import numpy as np

def PC2_to_numpyxyzarray(point_cloud):
    # create numpy array to store the point cloud
    point_cloud_array = np.zeros((point_cloud.width * point_cloud.height, 3))
    points = pc2.read_points(point_cloud, skip_nans=True)
    for i, p in enumerate(points):
        point_cloud_array[i, 0] = p[0]
        point_cloud_array[i, 1] = p[1]
        point_cloud_array[i, 2] = p[2]
    return point_cloud_array


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
        table_point_cloud_in_world = PC2_to_numpyxyzarray(req.table_point_cloud)
        segmented_point_cloud_in_world = PC2_to_numpyxyzarray(req.segmented_point_cloud)
        # convert point clouds to homogeneous coordinates
        table_point_cloud_in_world = np.concatenate((table_point_cloud_in_world, np.ones((table_point_cloud_in_world.shape[0], 1))), axis=1)
        segmented_point_cloud_in_world = np.concatenate((segmented_point_cloud_in_world, np.ones((segmented_point_cloud_in_world.shape[0], 1))), axis=1)
        rot_mat = R.from_quat([req.camera_stamped_transform.transform.rotation.x, req.camera_stamped_transform.transform.rotation.y, \
                            req.camera_stamped_transform.transform.rotation.z, req.camera_stamped_transform.transform.rotation.w]).as_matrix()
        camera_pose_mat = np.eye(4)
        camera_pose_mat[:3, :3] = rot_mat
        camera_pose_mat[:3, 3] = np.array([req.camera_stamped_transform.transform.translation.x, req.camera_stamped_transform.transform.translation.y, \
                                        req.camera_stamped_transform.transform.translation.z])

        # convert to camera frame
        table_point_cloud_in_camera = np.dot(np.linalg.inv(camera_pose_mat), table_point_cloud_in_world.T).T[:, :3]
        segmented_point_cloud_in_camera = np.dot(np.linalg.inv(camera_pose_mat), segmented_point_cloud_in_world.T).T[:, :3]
        # concate the two point clouds and create a 2d label array
        labels = np.zeros((table_point_cloud_in_camera.shape[0] + segmented_point_cloud_in_camera.shape[0], 1))
        labels[:table_point_cloud_in_camera.shape[0]] = 1
        point_cloud = np.concatenate((table_point_cloud_in_camera, segmented_point_cloud_in_camera), axis=0)
        # concate the point cloud with the label array
        point_cloud_labels = np.concatenate((point_cloud, labels), axis=1)
        # shuffle the point cloud and label array
        np.random.shuffle(point_cloud_labels)
        # if there are more points than num_point, randomly sample num_point points
        if point_cloud_labels.shape[0] > NUM_POINT:
            idx = np.random.choice(point_cloud_labels.shape[0], NUM_POINT, replace=False)
            point_cloud_labels = point_cloud_labels[idx, :]
        elif point_cloud_labels.shape[0] < NUM_POINT:
            # if there are less points than num_point, repeat the points then shuffle
            idx = np.random.choice(point_cloud_labels.shape[0], NUM_POINT - point_cloud_labels.shape[0], replace=True)
            point_cloud_labels = np.concatenate((point_cloud_labels, point_cloud_labels[idx, :]), axis=0)
            np.random.shuffle(point_cloud_labels)
        # reshape the point cloud and label array to (BATCH_SIZE, NUM_POINT, 4)
        point_cloud_labels = point_cloud_labels.reshape((BATCH_SIZE, NUM_POINT, 4))
        CoM = self.wrapped_model.predict(point_cloud_labels)
        CoM = np.dot(camera_pose_mat, np.append(CoM, 1))[:3]
        # convert the CoM to a Point message
        CoM_msg = Point()
        CoM_msg.x = CoM[0]
        CoM_msg.y = CoM[1]
        CoM_msg.z = CoM[2]
        return ComPredictResponse(com = CoM_msg)

def main():
    rospy.init_node("CoMPredict")
    rospy.loginfo("Creating the Tensorflow model")
    ri = RosInterface()
    rospy.loginfo("CoMPredict node initialized at service: CoMPredict")
    rospy.spin()

if __name__ == "__main__":
    main()
