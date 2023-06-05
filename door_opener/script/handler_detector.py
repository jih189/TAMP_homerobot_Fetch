#!/usr/bin/env python3
import rospy
import rospkg
from sensor_msgs.msg import Image, PointCloud, PointCloud2
from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pc2
import cv2
import numpy as np
import sys
from matplotlib import pyplot as plt, image, pyplot as plt
import open3d
from sensor_msgs.msg import CameraInfo
from shape_msgs.msg import Plane
from door_opener.srv import HandleDetection, HandleDetectionResponse
from door_opener.msg import HandleMotion
from scipy.spatial.transform import Rotation as R

class HandleDetectionServer:
    def __init__(self):
        darknet_dir = rospkg.RosPack().get_path('door_opener') + '/script/darknet/'
        self.model, self.classes, self.colors, self.output_layers  = self.load_yolo(darknet_dir)

        self.service = rospy.Service('handle_detection', HandleDetection, self.handle_object_detection)

    def imgmsg_to_cv2(self, img_msg):
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv

    def cv2_to_imgmsg(self, cv_image):
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
        return img_msg

    def load_yolo(self, model_dir):
        net = cv2.dnn.readNet(model_dir + "yolo-obj.weights", model_dir + "yolo-obj.cfg")
        classes = []
        with open(model_dir + "obj.names", "r") as f:
            classes = [line.strip() for line in f.readlines()]
        layers_names = net.getLayerNames()
        output_layers = [layers_names[i-1] for i in net.getUnconnectedOutLayers()]
        colors = np.random.uniform(0, 255, size=(len(classes), 3))
        return net, classes, colors, output_layers

    def detect_objects(self, img, net, outputLayers):
        blob = cv2.dnn.blobFromImage(img, scalefactor=0.00392, size=(320, 320), mean=(0, 0, 0), swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward(outputLayers)
        return blob, outputs

    def get_box_dimensions(self, outputs, height, width):
        boxes = []
        confs = []
        class_ids = []
        for output in outputs:
            for detect in output:
                scores = detect[5:]
                class_id = np.argmax(scores)
                conf = scores[class_id]
                if conf > 0.3:
                    center_x = int(detect[0] * width)
                    center_y = int(detect[1] * height)
                    w = int(detect[2] * width)
                    h = int(detect[3] * height)
                    x = int(center_x - w/2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confs.append(float(conf))
                    class_ids.append(class_id)
        return boxes, confs, class_ids

    def draw_labels(self, boxes, confs, colors, class_ids, classes, img): 
        import random
        indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                color = [random.randint(0, 255),random.randint(0, 255),random.randint(0, 255)]
                cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)
                cv2.putText(img, label, (x, y - 5), font, 1, [255, 255, 255], 1)
        plt.figure(figsize=(10, 10))
        plt.imshow(img)
        plt.title('my picture')
        plt.show()
        
    def cluster_boxes(self, boxes, confs, class_ids, classes):
        indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
        result = []
        for i in range(len(boxes)):
            if i in indexes:
                result.append([boxes[i], str(classes[class_ids[i]])])
        return result

    def convert_pointcloud2_to_pc(self, pointcloud2_msg):
        pc_data = pc2.read_points(pointcloud2_msg, skip_nans=True, field_names=("x", "y", "z"))
        pc_list = []

        for p in pc_data:
            pc_list.append([p[0], p[1], p[2]])

        pc_array = np.array(pc_list, dtype=np.float32)

        return pc_array

    def filter_points(self, pointcloud, camera_matrix, bbox):
        
        # Project points onto image plane
        projected_points = np.dot(pointcloud, camera_matrix.T)
        
        # Convert from homogeneous to 2D coordinates
        x = projected_points[:, 0] / projected_points[:, 2]
        y = projected_points[:, 1] / projected_points[:, 2]

        # Define bounding box limits
        bx, by, bw, bh = bbox
        xmin = bx
        ymin = by
        xmax = bx + bw
        ymax = by + bh
        
        # Check which points lie within the bounding box
        mask = (x >= xmin) & (x <= xmax) & (y >= ymin) & (y <= ymax)
        
        # Return only the points within the bounding box
        return pointcloud[mask], pointcloud[~mask]

    def keep_point_on_plane(self, plane_model, pointcloud):
        A, B, C, D = plane_model
        # Compute the distances from each point to the plane
        distances = np.abs(A * pointcloud[:, 0] + B * pointcloud[:, 1] + C * pointcloud[:, 2] + D) / np.sqrt(A**2 + B**2 + C**2)
        # Define the distance threshold - points with distance less than this are considered to be on the plane
        threshold = 0.01
        # Select points where the distance is less than the threshold
        return pointcloud[distances < threshold]

    def get_door_direction(self, plane_model, handle_point):
        door_direction = plane_model[:3] / np.linalg.norm(plane_model[:3])
        if door_direction.dot(handle_point) > 0:
            return -door_direction
        return door_direction

    def convert_from_numpy_to_msg(self, pointcloud):
        point_cloud_msg = PointCloud()
        point_cloud_msg.header.frame_id = 'base_link'
        for point in pointcloud:
            point_msg = Point32()
            point_msg.x = point[0]
            point_msg.y = point[1]
            point_msg.z = point[2]
            point_cloud_msg.points.append(point_msg)
        return point_cloud_msg

    def transform_plane(self, plane, pose_matrix):
        point_on_plane = np.array([-plane[3]/plane[0], 0, 0])

        normal = np.array([plane[0], plane[1], plane[2]])

        rotated_normal = np.dot(pose_matrix[:3,:3], normal)

        point_on_plane_homo = np.append(point_on_plane, 1)
        transformed_point_homo = np.dot(pose_matrix, point_on_plane_homo)
        transformed_point = transformed_point_homo[:3] / transformed_point_homo[3]

        return np.append(rotated_normal, -np.dot(rotated_normal, transformed_point))


    # def transform_plane(self, plane, pose_matrix):
    #     """
    #     Transforms a plane with a pose matrix.

    #     :param plane: Plane parameters [A, B, C, D] of the plane equation Ax + By + Cz + D = 0
    #     :type plane: numpy.ndarray
    #     :param pose_matrix: 4x4 pose matrix
    #     :type pose_matrix: numpy.ndarray
    #     :return: New plane parameters
    #     :rtype: numpy.ndarray
    #     """
    #     # Extract the normal vector and distance from the plane parameters
    #     normal = plane[:3]
    #     distance = plane[3]

    #     # Convert the normal vector to homogeneous coordinates
    #     normal_homogeneous = np.append(normal, 0)  # No translation for normal vector

    #     # Apply the pose matrix to the normal vector
    #     new_normal = np.dot(pose_matrix, normal_homogeneous)[:3]

    #     # Compute the new distance
    #     # Since the plane equation is Ax + By + Cz + D = 0, 
    #     # D can be calculated by substituting a point on the plane (x, y, z) into the equation.
    #     # Assume the point on the original plane is (0, 0, -D/C) if C!=0 or (0, -D/B, 0) if B!=0 or (-D/A, 0, 0) if A!=0,
    #     # we first rotate this point with the rotation part of the pose matrix, then calculate the new D value 
    #     if normal[2] != 0:
    #         original_point = np.array([0, 0, -distance / normal[2], 1])
    #     elif normal[1] != 0:
    #         original_point = np.array([0, -distance / normal[1], 0, 1])
    #     else:  # normal[0] != 0
    #         original_point = np.array([-distance / normal[0], 0, 0, 1])
    #     new_point = np.dot(pose_matrix, original_point)
    #     new_distance = -new_normal.dot(new_point[:3])

    #     return np.append(new_normal, new_distance)

    def handle_object_detection(self, req):

        rot_mat = R.from_quat([req.camera_stamped_transform.transform.rotation.x, req.camera_stamped_transform.transform.rotation.y, \
                            req.camera_stamped_transform.transform.rotation.z, req.camera_stamped_transform.transform.rotation.w]).as_matrix()
        camera_pose_mat = np.eye(4)
        camera_pose_mat[:3, :3] = rot_mat
        camera_pose_mat[:3, 3] = np.array([req.camera_stamped_transform.transform.translation.x, req.camera_stamped_transform.transform.translation.y, \
                                        req.camera_stamped_transform.transform.translation.z])

        # get camera info
        camera_info = rospy.wait_for_message('/head_camera/rgb/camera_info', CameraInfo)
        camera_matrix = np.reshape(camera_info.K, (3, 3))
        
        # get color image from camera
        image = rospy.wait_for_message("/head_camera/rgb/image_rect_color", Image)
        # get pointcloud from camera
        pointcloud_raw = rospy.wait_for_message("/head_camera/depth_downsample/points", PointCloud2)

        cv2_img = cv2.cvtColor(self.imgmsg_to_cv2(image), cv2.COLOR_BGR2RGB)
        pointcloud = self.convert_pointcloud2_to_pc(pointcloud_raw)

        # estimate the handle.
        blob, outputs = self.detect_objects(cv2_img, self.model, self.output_layers)
        boxes, confs, class_ids = self.get_box_dimensions(outputs, cv2_img.shape[0], cv2_img.shape[1])

        res = HandleDetectionResponse()
        number_of_handles = 0

        for b, n in self.cluster_boxes(boxes, confs, class_ids, self.classes):
            if n == 'handle':
                number_of_handles += 1
                filter_pointcloud, rest_pointcloud = self.filter_points(pointcloud, camera_matrix, b)
                
                handle_point = np.mean(filter_pointcloud, axis=0)
                
                # get the rest point close to this mean point
                distances = np.sqrt(np.sum((rest_pointcloud - handle_point)**2, axis=1))
                radius_points = rest_pointcloud[distances < 0.2]

                # do plane fitting
                pcd = open3d.geometry.PointCloud()
                pcd.points = open3d.utility.Vector3dVector(radius_points)

                # Perform plane segmentation
                plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                        ransac_n=3,
                                                        num_iterations=1000)

                door_pc = self.keep_point_on_plane(plane_model, rest_pointcloud)
                door_direction = self.get_door_direction(plane_model, handle_point)
                door_goal_point = handle_point + door_direction
                handle_motion = HandleMotion()

                # convert the handle point to the world frame
                handle_point = np.dot(camera_pose_mat, np.append(handle_point, 1))[:3]

                # convert the handle point to the world frame
                door_goal_point = np.dot(camera_pose_mat, np.append(door_goal_point, 1))[:3]
                door_direction = door_goal_point - handle_point

                handle_motion.handle_direction = np.concatenate((handle_point, door_direction)).tolist()

                # Convert the point to world frame
                filter_pointcloud = np.dot(camera_pose_mat, np.hstack((filter_pointcloud, np.ones((filter_pointcloud.shape[0], 1)))).T).T[:, :3]
                handle_motion.handle_pc = self.convert_from_numpy_to_msg(filter_pointcloud)

                # rotate the plane into the base link
                handle_motion.drawer_plane.coef = self.transform_plane(plane_model, camera_pose_mat)

                # handle_directions.append(np.concatenate((handle_point, door_direction)))
                # handle_pcs.append(filter_pointcloud)

                res.handle_motions.append(handle_motion)

        # # Convert the point to world frame
        pointcloud = np.dot(camera_pose_mat, np.hstack((pointcloud, np.ones((pointcloud.shape[0], 1)))).T).T[:, :3]
        res.full_pc = self.convert_from_numpy_to_msg(pointcloud)

        print("detect ", number_of_handles, " handles from the scene.")

        # construct the solution
        return res

if __name__ == "__main__":
    rospy.init_node('handle_detection_server')
    server = HandleDetectionServer()
    print("door handle detection server is launching!")
    rospy.spin()