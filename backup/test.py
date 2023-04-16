import bpy
import numpy as np
import sys
import os
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

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
        bpy.context.scene.render.filepath = "rendered_image.png"

    def publish_image(self):
        while not rospy.is_shutdown():
            try:
                # Render the scene
                bpy.ops.render.render(write_still=True)

                render_result = bpy.data.images['Render Result']

                image = cv2.imread(bpy.context.scene.render.filepath)
                #image = cv2.imread("rendered_image.png")

                ros_image = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")

            except Exception as e:
                print(e)
                break

            self.pub.publish(ros_image)


if __name__ == '__main__':
    rospy.init_node('blender_cam_publisher', anonymous=True)

    image_publisher = ImagePublisher()

    # redirect output to log file
    logfile = 'blender_render.log'
    open(logfile, 'a').close()
    old = os.dup(sys.stdout.fileno())
    sys.stdout.flush()
    os.close(sys.stdout.fileno())
    fd = os.open(logfile, os.O_WRONLY)

    #try:
    image_publisher.publish_image()
    #    pass
    #except rospy.ROSInterruptException:
    #    pass

    # disable output redirection
    os.close(fd)
    os.dup(old)
    os.close(old)
