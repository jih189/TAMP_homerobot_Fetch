#! /usr/bin/python

import numpy as np
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import cv2

from rail_manipulation_msgs.msg import SegmentedObjectList, SegmentedObject

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

class ObjectSegmentation:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = "/head_camera/seg/image_rect"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.seg_topic = "/head_camera/seg/image_rect_color"
        self.image_pub = rospy.Publisher(self.seg_topic, Image, queue_size=10)
        self.segmented_object_info_topic = "/segmented_object_list"
        self.segmented_object_info_pub = rospy.Publisher(self.segmented_object_info_topic, SegmentedObjectList, queue_size=1)

    def setObjectHandleIds(self, handles, object_names):
        self.object_handles = handles
        self.object_colors = [np.random.choice(range(256), size=3) for _ in range(len(handles))]
        self.object_names = object_names

        # prepare the segmented object list.
        self.segmented_object_list = SegmentedObjectList()
        for color, name in zip(self.object_colors, self.object_names):
            segmented_object = SegmentedObject()
            segmented_object.rgb = color[::-1] # the color need to be changed to bgr.
            segmented_object.name = name
            self.segmented_object_list.objects.append(segmented_object)

    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            data = cv2_img.copy()
            data[:,:,0] *= (256*256)
            data[:,:,1] *= 256
            data = np.sum(data, axis=2)
            resultimg = np.zeros(cv2_img.shape, dtype=np.uint8)
            for j in range(len(self.object_handles)):
                resultimg[data==self.object_handles[j]] = self.object_colors[j]
            output_image = self.bridge.cv2_to_imgmsg(resultimg, "bgr8")
            output_image.header = msg.header
            self.image_pub.publish(output_image)

            # need to publish the segmented object list as well
            self.segmented_object_info_pub.publish(self.segmented_object_list)

        except CvBridgeError, e:
            print(e)


if __name__ == '__main__':
    # main()
    rospy.init_node('object_segmentation')
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    if clientID!=-1:
        

        print ('Connected to remote API server')

        # get all visible desired objects
        res, allObjectIds, intData, floatData, stringData = sim.simxGetObjectGroupData(clientID, sim.sim_object_shape_type, 0, sim.simx_opmode_blocking)
        packed_res = [(i, s[7: -7]) for i, s in enumerate(stringData) if s.startswith('object_') and s.endswith('_visual')]
        if len(packed_res) > 0:
            desired_object_indices, desired_object_names = zip(*packed_res)
            desired_object_indices = list(desired_object_indices)
            desired_object_names = list(desired_object_names)
            
            handleIds = [allObjectIds[index] for index in desired_object_indices]
        else:
            handleIds = []
            desired_object_names = []

        object_segementation = ObjectSegmentation()
        object_segementation.setObjectHandleIds(handleIds, desired_object_names)

        rospy.spin()

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
