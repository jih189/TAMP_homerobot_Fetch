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
import tf.transformations as tf_trans

class ImagePublisher:
    def __init__(self, topic='blender_camera/image_raw', rate=10):
        print("initialize the image publisher")
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

        self.fetch_robot_arm_objects = {}
        self.fetch_part_pose_in_link = {}

        # try to load the robot fingers into the blender
        bpy.ops.import_scene.obj(filepath="robot_meshes/l_gripper_finger_link.obj")
        self.fetch_robot_arm_objects['l_gripper_finger_link'] = bpy.context.selected_objects[0]
        self.fetch_robot_arm_objects['l_gripper_finger_link'].rotation_mode = 'QUATERNION'
        self.fetch_part_pose_in_link['l_gripper_finger_link'] = ((0,-0.101425,0),(0,0,0,1))

        bpy.ops.import_scene.obj(filepath="robot_meshes/r_gripper_finger_link.obj")
        self.fetch_robot_arm_objects['r_gripper_finger_link'] = bpy.context.selected_objects[0]
        self.fetch_robot_arm_objects['r_gripper_finger_link'].rotation_mode = 'QUATERNION'
        self.fetch_part_pose_in_link['r_gripper_finger_link'] = ((0,0.101425,0),(0,0,0,1))

        bpy.ops.import_scene.obj(filepath="robot_meshes/gripper_link.obj")
        self.fetch_robot_arm_objects['gripper_link'] = bpy.context.selected_objects[0]
        self.fetch_robot_arm_objects['gripper_link'].rotation_mode = 'QUATERNION'
        self.fetch_part_pose_in_link['gripper_link'] = ((0,0,0),(0,0,0,1))

        bpy.ops.wm.collada_import(filepath="robot_meshes/wrist_roll_link.dae")
        self.fetch_robot_arm_objects['wrist_roll_link'] = bpy.context.selected_objects[0]
        self.fetch_robot_arm_objects['wrist_roll_link'].rotation_mode = 'QUATERNION'
        self.fetch_part_pose_in_link['wrist_roll_link'] = ((0,0,0),(0,0,0,1))

        bpy.ops.wm.collada_import(filepath="robot_meshes/wrist_flex_link.dae")
        self.fetch_robot_arm_objects['wrist_flex_link'] = bpy.context.selected_objects[0]
        self.fetch_robot_arm_objects['wrist_flex_link'].rotation_mode = 'QUATERNION'
        self.fetch_part_pose_in_link['wrist_flex_link'] = ((0,0,0),(0,0,0,1))

        bpy.ops.wm.collada_import(filepath="robot_meshes/upperarm_roll_link.dae")
        self.fetch_robot_arm_objects['upperarm_roll_link'] = bpy.context.selected_objects[0]
        self.fetch_robot_arm_objects['upperarm_roll_link'].rotation_mode = 'QUATERNION'
        self.fetch_part_pose_in_link['upperarm_roll_link'] = ((0,0,0),(0,0,0,1))

        bpy.ops.wm.collada_import(filepath="robot_meshes/shoulder_pan_link.dae")
        self.fetch_robot_arm_objects['shoulder_pan_link'] = bpy.context.selected_objects[0]
        self.fetch_robot_arm_objects['shoulder_pan_link'].rotation_mode = 'QUATERNION'
        self.fetch_part_pose_in_link['shoulder_pan_link'] = ((0,0,0),(0,0,0,1))

        bpy.ops.wm.collada_import(filepath="robot_meshes/shoulder_lift_link.dae")
        self.fetch_robot_arm_objects['shoulder_lift_link'] = bpy.context.selected_objects[0]
        self.fetch_robot_arm_objects['shoulder_lift_link'].rotation_mode = 'QUATERNION'
        self.fetch_part_pose_in_link['shoulder_lift_link'] = ((0,0,0),(0,0,0,1))

        bpy.ops.wm.collada_import(filepath="robot_meshes/forearm_roll_link.dae")
        self.fetch_robot_arm_objects['forearm_roll_link'] = bpy.context.selected_objects[0]
        self.fetch_robot_arm_objects['forearm_roll_link'].rotation_mode = 'QUATERNION'
        self.fetch_part_pose_in_link['forearm_roll_link'] = ((0,0,0),(0,0,0,1))

        bpy.ops.wm.collada_import(filepath="robot_meshes/elbow_flex_link.dae")
        self.fetch_robot_arm_objects['elbow_flex_link'] = bpy.context.selected_objects[0]
        self.fetch_robot_arm_objects['elbow_flex_link'].rotation_mode = 'QUATERNION'
        self.fetch_part_pose_in_link['elbow_flex_link'] = ((0,0,0),(0,0,0,1))


        print("blender image publisher is ready!")

    def render_object_in_scene(self, world_frame_id, object_frame_id, desired_object, object_pose_in_link):
        try:
            object_pose_in_link_mat = tf_trans.quaternion_matrix(object_pose_in_link[1])
            object_pose_in_link_mat[:3, 3] = np.array(object_pose_in_link[0])
            trans = self.tfBuffer.lookup_transform(world_frame_id, object_frame_id, rospy.Time())

            link_in_world_mat = tf_trans.quaternion_matrix([trans.transform.rotation.x, 
                                                            trans.transform.rotation.y, 
                                                            trans.transform.rotation.z, 
                                                            trans.transform.rotation.w])
            link_in_world_mat[:3, 3] = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            object_pose_in_world_mat = np.dot(link_in_world_mat, object_pose_in_link_mat)
            object_quaternion = tf_trans.quaternion_from_matrix(object_pose_in_world_mat)

            desired_object.location = (object_pose_in_world_mat[0, 3], object_pose_in_world_mat[1, 3], object_pose_in_world_mat[2, 3])
            desired_object.rotation_quaternion = mathutils.Quaternion((object_quaternion[3],
                                                                       object_quaternion[0],
                                                                       object_quaternion[1],
                                                                       object_quaternion[2]))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.rate.sleep()

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

            for robot_object in self.fetch_robot_arm_objects:
                self.render_object_in_scene('world', robot_object, 
                                                self.fetch_robot_arm_objects[robot_object], 
                                                self.fetch_part_pose_in_link[robot_object])
            
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
