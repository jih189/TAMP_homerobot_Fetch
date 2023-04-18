#!/usr/bin/env python3

import rospy 
import numpy as np
from ros_tensorflow_msgs.srv import ComPredict, ComPredictRequest, ComPredictResponse
from sensor_msgs.msg import PointCloud2
import ros_numpy
import geometry_msgs.msg

# initialize the node
rospy.init_node('test_tensorflow_services')


# generate a dummy point cloud with 100 points as a structured numpy array
point_cloud = np.zeros((100,), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
point_cloud['x'] = np.random.rand(100)
point_cloud['y'] = np.random.rand(100)
point_cloud['z'] = np.random.rand(100)

# convert the point cloud to a ROS message
point_cloud_msg = ros_numpy.msgify(PointCloud2, point_cloud, stamp=rospy.Time.now(), frame_id='camera_link')

# generate a dummy table point cloud with 100 points as a structured numpy array
table_point_cloud = np.zeros((100,), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
table_point_cloud['x'] = np.random.rand(100)
table_point_cloud['y'] = np.random.rand(100)
table_point_cloud['z'] = np.random.rand(100)

# convert the point cloud to a ROS message
table_point_cloud_msg = ros_numpy.msgify(PointCloud2, table_point_cloud, stamp=rospy.Time.now(), frame_id='camera_link')

# generate a dummy camera transform
camera_stamped_transform = geometry_msgs.msg.TransformStamped()
camera_stamped_transform.header.stamp = rospy.Time.now()
camera_stamped_transform.header.frame_id = 'camera_link'
camera_stamped_transform.child_frame_id = 'camera_link'
camera_stamped_transform.transform.translation.x = 0.0
camera_stamped_transform.transform.translation.y = 0.0
camera_stamped_transform.transform.translation.z = 0.0
camera_stamped_transform.transform.rotation.x = 0.0
camera_stamped_transform.transform.rotation.y = 0.0
camera_stamped_transform.transform.rotation.z = 0.0
camera_stamped_transform.transform.rotation.w = 1.0



# # create a request object
# req = ComPredictRequest()
# req.segmented_point_cloud = point_cloud_msg
# req.table_point_cloud = table_point_cloud_msg
# req.camera_stamped_transform = camera_stamped_transform


# # wait for the service to be available
# rospy.wait_for_service('CoMPredict')

# # create a handle to the service
# predict = rospy.ServiceProxy('CoMPredict', ComPredict)

# # call the service
# res = predict(req)

# # print the response
# print(res)

# next, we test grasp prediction

from ros_tensorflow_msgs.srv import Predict, PredictRequest, PredictResponse

# wait for the service to be available
rospy.wait_for_service('grasp_predict')

# create a handle to the service
predict = rospy.ServiceProxy('grasp_predict', Predict)

# create a request object
req = PredictRequest()

# fill the request
req.full_point_cloud = point_cloud_msg
req.segmented_point_cloud = point_cloud_msg
req.camera_stamped_transform = camera_stamped_transform

# call the service
res = predict(req)

# print the response
print(res)


