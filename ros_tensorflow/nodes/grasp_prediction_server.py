#!/usr/bin/env python3

import rospy
from ros_tensorflow_msgs.srv import Predict, PredictRequest, PredictResponse
from geometry_msgs.msg import PoseStamped

from contact_graspnet.contact_graspnet.model import ModelWrapper
import ros_numpy
from scipy.spatial.transform import Rotation as R

import numpy as np

class RosInterface():
    def __init__(self):
        self.wrapped_model = ModelWrapper()
        self.predict_srv = rospy.Service('grasp_predict', Predict, self.predict_cb)

    def predict_cb(self, req):
        rospy.loginfo("get data with camera pose")
        full_point_cloud_in_world = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(req.full_point_cloud)
        # convert it to homogeneous coordinates
        full_point_cloud_in_world = np.insert(full_point_cloud_in_world, full_point_cloud_in_world.shape[1], 1.0, axis=1)
        segmented_point_cloud_in_world = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(req.segmented_point_cloud)
        # convert it to homogeneous coordinates
        segmented_point_cloud_in_world = np.insert(segmented_point_cloud_in_world, segmented_point_cloud_in_world.shape[1], 1.0, axis=1)
        rot_mat = R.from_quat([req.camera_stamped_transform.transform.rotation.x, req.camera_stamped_transform.transform.rotation.y, \
                            req.camera_stamped_transform.transform.rotation.z, req.camera_stamped_transform.transform.rotation.w]).as_matrix()
        camera_pose_mat = np.eye(4)
        camera_pose_mat[:3, :3] = rot_mat
        camera_pose_mat[:3, 3] = np.array([req.camera_stamped_transform.transform.translation.x, req.camera_stamped_transform.transform.translation.y, \
                                        req.camera_stamped_transform.transform.translation.z])

        full_point_cloud_in_camera = np.dot(np.linalg.inv(camera_pose_mat), full_point_cloud_in_world.T).T[:, :3]
        segmented_point_cloud_in_camera = np.dot(np.linalg.inv(camera_pose_mat), segmented_point_cloud_in_world.T).T[:, :3]

        grasp_poses_in_cam, grasp_scores = self.wrapped_model.predict(full_point_cloud_in_camera, segmented_point_cloud_in_camera)

        predict_result = PredictResponse()
        to_fetch_pose = np.eye(4)
        to_fetch_pose[:3, :3] = R.from_quat([0.5, -0.5, 0.5, 0.5]).as_matrix()
        to_fetch_pose[:3, 3] = np.array([0.0, 0.0, -0.09])
        for grasp_pose_in_cam, score in zip(grasp_poses_in_cam, grasp_scores):
            grasp_pose_in_world_Panda = np.dot(camera_pose_mat, grasp_pose_in_cam)

            # need to rotate it as fetch gripper.
            grasp_pose_in_world = grasp_pose_in_world_Panda.dot(to_fetch_pose)

            grasp_rot_quaterion = R.from_matrix(grasp_pose_in_world[:3, :3]).as_quat()
            predicted_grasp_pose = PoseStamped()
            predicted_grasp_pose.header.frame_id = req.camera_stamped_transform.header.frame_id
            predicted_grasp_pose.header.stamp = rospy.Time.now()

            predicted_grasp_pose.pose.position.x = grasp_pose_in_world[0, 3]
            predicted_grasp_pose.pose.position.y = grasp_pose_in_world[1, 3]
            predicted_grasp_pose.pose.position.z = grasp_pose_in_world[2, 3]

            predicted_grasp_pose.pose.orientation.x = grasp_rot_quaterion[0]
            predicted_grasp_pose.pose.orientation.y = grasp_rot_quaterion[1]
            predicted_grasp_pose.pose.orientation.z = grasp_rot_quaterion[2]
            predicted_grasp_pose.pose.orientation.w = grasp_rot_quaterion[3]

            predict_result.predicted_grasp_poses.append(predicted_grasp_pose)
            predict_result.scores.append(score)

        return predict_result


def main():
    rospy.init_node("ros_tensorflow")
    rospy.loginfo("Creating the Tensorflow model")
    ri = RosInterface()
    rospy.loginfo("ros_tensorflow node initialized")
    rospy.spin()

if __name__ == "__main__":
    main()
