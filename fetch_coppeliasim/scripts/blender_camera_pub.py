import bpy
import numpy as np
import sys
import os
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import tf2_ros
import math
import mathutils

class ImagePublisher:
    def __init__(self, topic='blender_camera/image_raw', rate=10):
        # Set the resolution of the output image
        bpy.context.scene.render.resolution_x = 640
        bpy.context.scene.render.resolution_y = 480

        # Set the render settings
        bpy.context.scene.render.image_settings.file_format = 'PNG'
        bpy.context.scene.render.image_settings.color_depth = '8'
        
        self.pub = rospy.Publisher(topic, Image, queue_size=10)
        self.bridge = CvBridge()
        bpy.context.scene.render.filepath = "/root/rendered_image.png"
        self.rate = rospy.Rate(rate)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # set camera's field of view
        desired_fov_radians = math.radians(60)
        self.camera = bpy.data.objects['Camera']
        self.camera.data.lens = (0.5 * self.camera.data.sensor_width) / math.tan(0.5 * desired_fov_radians)
        # set camera's near and far clipping planes
        self.camera.data.clip_start = 0.16 # near clipping plane 
        self.camera.data.clip_end = 10 # far clipping plane
        self.camera.rotation_mode = 'QUATERNION'
        # prepare later for rotate the camera.s
        self.rotation_quaternion = mathutils.Quaternion([1,0,0], math.pi)

    def publish_image(self):
        while not rospy.is_shutdown():
            # get the transform from world to camera
            try:
                trans = self.tfBuffer.lookup_transform('world', 'head_camera_rgb_optical_frame', rospy.Time())
                # set camera location
                self.camera.location = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
                camera_rotation = mathutils.Quaternion((trans.transform.rotation.w,
                                                        trans.transform.rotation.x,
                                                        trans.transform.rotation.y,
                                                        trans.transform.rotation.z))
                self.camera.rotation_quaternion = camera_rotation @ self.rotation_quaternion 
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            try:
                # Render the scene
                bpy.ops.render.render(write_still=True)

                render_result = bpy.data.images['Render Result']

                image = cv2.imread(bpy.context.scene.render.filepath)

                ros_image = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")

            except Exception as e:
                print(e)
                break

            self.pub.publish(ros_image)


if __name__ == '__main__':
    rospy.init_node('blender_cam_publisher', anonymous=True)

    image_publisher = ImagePublisher()

    # redirect output to log file
    logfile = '/root/blender_render.log'
    open(logfile, 'a').close()
    old = os.dup(sys.stdout.fileno())
    sys.stdout.flush()
    os.close(sys.stdout.fileno())
    fd = os.open(logfile, os.O_WRONLY)

    try:
        image_publisher.publish_image()
    except rospy.ROSInterruptException:
        pass

    # disable output redirection
    os.close(fd)
    os.dup(old)
    os.close(old)
