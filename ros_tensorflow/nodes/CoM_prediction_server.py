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
from rosnumpy_Com import pointcloud2_to_xyz_array
from scipy.spatial.transform import Rotation as R

import numpy as np

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
        table_point_cloud_in_world = pointcloud2_to_xyz_array(req.table_point_cloud)
        segmented_point_cloud_in_world = pointcloud2_to_xyz_array(req.segmented_point_cloud)
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
        # print("segmented_point_cloud_in_camera.shape: ", segmented_point_cloud_in_camera.shape)

        # print(segmented_point_cloud_in_camera)
        # print(table_point_cloud_in_camera)

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
        # use open3d to show the point cloud
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud_labels[:, :3])
        # paint the points with their labels
        pcd.colors = o3d.utility.Vector3dVector(np.array([[1, 0, 0] if label == 1 else [0, 1, 0] for label in point_cloud_labels[:, 3]]))
        o3d.visualization.draw_geometries([pcd])

        # reshape the point cloud and label array to (BATCH_SIZE, NUM_POINT, 4)
        point_cloud_labels = point_cloud_labels.reshape((BATCH_SIZE, NUM_POINT, 4))
        CoM = self.wrapped_model.predict(point_cloud_labels)
        CoM = np.dot(camera_pose_mat, np.append(CoM, 1))[:3]
        # use open3d to show the point cloud and the CoM
        # draw the CoM as a axis
        pcd.paint_uniform_color([1, 0, 0])
        pcd2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=CoM)

        o3d.visualization.draw_geometries([pcd, pcd2])

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
