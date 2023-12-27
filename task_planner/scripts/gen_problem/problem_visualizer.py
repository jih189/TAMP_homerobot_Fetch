# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point

class ProblemVisualizer(object):
    def __init__(self, publisher):
        self.publisher = publisher

    def visualize_obstacle(self, env_pose, env_mesh_path):
        marker_array = MarkerArray()

        obstacle_marker = Marker()
        obstacle_marker.header.frame_id = "base_link"
        obstacle_marker.header.stamp = rospy.Time.now()
        obstacle_marker.ns = "obstacle"
        obstacle_marker.id = 0
        obstacle_marker.type = Marker.MESH_RESOURCE
        obstacle_marker.action = Marker.ADD
        obstacle_marker.pose = env_pose.pose
        obstacle_marker.scale = Point(1, 1, 1)
        obstacle_marker.color = ColorRGBA(0.5, 0.5, 0.5, 1)
        obstacle_marker.mesh_resource = "package://task_planner/mesh_dir/" + os.path.basename(env_mesh_path)
        marker_array.markers.append(obstacle_marker)

        self.publisher.publish(marker_array)

    def visualize_placements(self, feasible_placements, manipulated_object_mesh_path):
        marker_array = MarkerArray()

        for i, placement in enumerate(feasible_placements):
            object_marker = Marker()
            object_marker.header.frame_id = "base_link"
            object_marker.header.stamp = rospy.Time.now()
            object_marker.ns = "placement"
            object_marker.id = i + 1
            object_marker.type = Marker.MESH_RESOURCE
            object_marker.action = Marker.ADD
            object_marker.pose = self._convert_matrix_to_pose(placement)
            object_marker.scale = Point(1, 1, 1)
            object_marker.color = ColorRGBA(0.5, 0.5, 0.5, 1)
            object_marker.mesh_resource = "package://task_planner/mesh_dir/" + os.path.basename(manipulated_object_mesh_path)
            marker_array.markers.append(object_marker)

        self.publisher.publish(marker_array)

    @staticmethod
    def _convert_matrix_to_pose(matrix):
        # 这里假设matrix是一个4x4的numpy数组，代表一个变换矩阵
        pose = Pose()
        # ... [转换矩阵到Pose的代码] ...
        # 例如，使用tf.transformations库来从变换矩阵中提取位置和四元数
        return pose
