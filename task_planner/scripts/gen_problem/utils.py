def convert_pose_stamped_to_matrix(pose_stamped):
    pose_matrix = transformations.quaternion_matrix([pose_stamped.pose.orientation.w, 
                                                    pose_stamped.pose.orientation.x, 
                                                    pose_stamped.pose.orientation.y, 
                                                    pose_stamped.pose.orientation.z])
    pose_matrix[0, 3] = pose_stamped.pose.position.x
    pose_matrix[1, 3] = pose_stamped.pose.position.y
    pose_matrix[2, 3] = pose_stamped.pose.position.z
    return pose_matrix