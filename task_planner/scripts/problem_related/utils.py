import sys

from trimesh import transformations
import numpy as np
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Point32

def convert_pose_stamped_to_matrix(pose_stamped):
    pose_matrix = transformations.quaternion_matrix([pose_stamped.pose.orientation.w,
                                                     pose_stamped.pose.orientation.x,
                                                     pose_stamped.pose.orientation.y,
                                                     pose_stamped.pose.orientation.z])
    pose_matrix[0, 3] = pose_stamped.pose.position.x
    pose_matrix[1, 3] = pose_stamped.pose.position.y
    pose_matrix[2, 3] = pose_stamped.pose.position.z
    return pose_matrix

def create_pose_stamped(pose_data):
    pose = PoseStamped()
    pose.header.frame_id = pose_data['frame_id']
    pose.pose.position.x = pose_data['position']['x']
    pose.pose.position.y = pose_data['position']['y']
    pose.pose.position.z = pose_data['position']['z']
    pose.pose.orientation.x = pose_data['orientation']['x']
    pose.pose.orientation.y = pose_data['orientation']['y']
    pose.pose.orientation.z = pose_data['orientation']['z']
    pose.pose.orientation.w = pose_data['orientation']['w']
    return pose

def get_position_difference_between_poses(pose_1_, pose_2_):
    '''
    Get the position difference between two poses.
    pose_1_ and pose_2_ are both 4x4 numpy matrices.
    '''
    return np.linalg.norm(pose_1_[:3, 3] - pose_2_[:3, 3])


def gaussian_similarity(distance, max_distance, sigma=0.01):
    """
    Calculate the similarity score using Gaussian function.
    distance: the distance between two configurations
    sigma: the sigma of the Gaussian function
    max_distance: the maximum distance between two configurations
    The score is between 0 and 1. The larger the score, the more similar the two configurations are.
    If sigma is heigher, the scope of the Gaussian function is wider.
    """
    if distance == 0:  # when the distance is 0, the score should be 1
        return 1.0

    # Calculate the similarity score using Gaussian function
    score = np.exp(-(distance ** 2) / (2 * sigma ** 2))
    max_score = np.exp(-(max_distance ** 2) / (2 * sigma ** 2))
    score = (score - max_score) / (1 - max_score)

    if score < 0.001:
        score = 0.0

    return score
