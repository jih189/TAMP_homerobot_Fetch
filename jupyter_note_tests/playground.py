import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import rospy
import tf2_ros
from ros_tensorflow_msgs.srv import *
from rail_segmentation.srv import *
from rail_manipulation_msgs.srv import *
from geometry_msgs.msg import TransformStamped, Pose
from manipulation_test.srv import *
import numpy as np
from ros_numpy import numpify, msgify
import tf
import sys
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
import random
from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import uuid
import sensor_msgs.point_cloud2 as pc2

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3

from tf2_sensor_msgs import do_transform_cloud

marker_array_publisher = rospy.Publisher('/grasp_markers', MarkerArray, queue_size=10)

def publish_markers(predict_response):
    marker_array_msg = MarkerArray()
    marker_array_publisher.publish(marker_array_msg)
    for i, grasp_pose in enumerate(predict_response.predicted_grasp_poses):
        marker_msg = Marker()
        marker_msg.header.frame_id = grasp_pose.header.frame_id or "base_link"
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.ns = "grasp_poses"
        marker_msg.id = i
        marker_msg.type = Marker.ARROW
        marker_msg.action = Marker.ADD
        marker_msg.pose = grasp_pose.pose
        
        marker_msg.scale = Vector3(0.01, 0.002, 0.002)

        marker_msg.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        
        marker_msg.lifetime = rospy.Duration(0)
        
        marker_array_msg.markers.append(marker_msg)
    
    marker_array_publisher.publish(marker_array_msg)
    rospy.loginfo("Published grasp markers to /grasp_markers")

def get_transform_matrix(frame_id_target, frame_id_source):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    if tf_buffer.can_transform(frame_id_target, frame_id_source, rospy.Time(0), rospy.Duration(4.0)):
        trans = tf_buffer.lookup_transform(frame_id_target, frame_id_source, rospy.Time(0))
    else:
        print("Unable to trans")
    translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
    rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
    
    rot_mat = tf.transformations.quaternion_matrix(rotation)

    transform_mat = np.eye(4)
    transform_mat[:3, :3] = rot_mat[:3, :3]
    transform_mat[:3, 3] = translation
    
    return transform_mat

def transform_point_cloud(pc2_msg, target_frame):

    
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.sleep(1.0)
    
    transform = tf_buffer.lookup_transform(target_frame,
                                           pc2_msg.header.frame_id, 
                                           rospy.Time(0),          
                                           rospy.Duration(1.0))   
    
    transformed_pc2_msg = do_transform_cloud(pc2_msg, transform)
    
    return transformed_pc2_msg


class ItemPCManager:
    def __init__(self):
        self.color_to_points = {}
        self.color_to_uuid = {}
        self.uuid_to_color = {}
        self.uuid_to_pc2 = {}
        self.color_to_pc2 = {}
    
    def reset(self):
        self.color_to_points = {}
        self.color_to_uuid = {}
        self.uuid_to_color = {}
        self.uuid_to_pc2 = {}
        self.color_to_pc2 = {}
    
    def add_item(self, r, g, b, point, point_cloud2):
        color_key = (r, g, b)
        if color_key != (0, 0, 0):
            print(color_key)
        if color_key not in self.color_to_points:
            self.color_to_points[color_key] = []
            new_uuid = str(uuid.uuid4())
            self.color_to_uuid[color_key] = new_uuid
            self.color_to_pc2[color_key] = point_cloud2
            self.uuid_to_color[new_uuid] = color_key
            self.uuid_to_pc2[new_uuid] = point_cloud2
        else:
            self.color_to_pc2[color_key] = self.merge_pc2(self.color_to_pc2[color_key], point_cloud2)
        if point not in self.color_to_points[color_key]:
            self.color_to_points[color_key].append(point)

    def merge_pc2(self, pc2_1, pc2_2):
        assert pc2_1.fields == pc2_2.fields
        assert pc2_1.point_step == pc2_2.point_step
        assert pc2_1.is_bigendian == pc2_2.is_bigendian

        merged_data = pc2_1.data + pc2_2.data

        merged_pc2 = PointCloud2()
        merged_pc2.header = pc2_1.header
        merged_pc2.height = max(pc2_1.height, pc2_2.height)
        merged_pc2.width = (len(merged_data) / pc2_1.point_step) / merged_pc2.height
        merged_pc2.fields = pc2_1.fields
        merged_pc2.is_bigendian = pc2_1.is_bigendian
        merged_pc2.point_step = pc2_1.point_step
        merged_pc2.row_step = merged_pc2.point_step * merged_pc2.width
        merged_pc2.data = merged_data
        merged_pc2.is_dense = pc2_1.is_dense and pc2_2.is_dense

        return merged_pc2

    def update_item(self, r, g, b, point):
        color_key = (r, g, b)
        #TODO
        
    def get_item_by_color(self, r, g, b):
        return self.color_to_points.get((r, g, b), [])
    
    def get_pc2_by_color(self, r, g, b):
        color_key = (r, g, b)
        return self.color_to_pc2.get((r, g, b), [])    

    def get_item_by_uuid(self, input_uuid):
        color_key = self.uuid_to_color.get(input_uuid)
        if color_key:
            return self.color_to_points[color_key]
        else:
            return []
    
    def get_pc2_by_uuid(self):
        print("")
        # TODO
    
    def list_items(self):
        for color, points in self.color_to_points.items():
            print("Color: {}, UUID: {}, Number of points: {}".format(color, self.color_to_uuid[color], len(points)))

should_continue = True

def filter_point_cloud_by_color(point_cloud_data, color, point_step):
    filtered_data = bytearray(len(point_cloud_data.data))

    for i in range(0, len(point_cloud_data.data), point_step):
        point_data = point_cloud_data.data[i:i+point_step]
        rgba = struct.unpack_from('I', point_data, 16)[0]
        r = (rgba >> 16) & 0xFF
        g = (rgba >> 8) & 0xFF
        b = rgba & 0xFF

        if (r, g, b) == color:
            filtered_data[i:i+point_step] = point_data
        else:
            nan_float = struct.pack('f', float('nan'))
            filtered_data[i:i+12] = nan_float * 3
            filtered_data[i+16:i+point_step] = point_data[16:point_step]

    filtered_pc2 = PointCloud2()
    filtered_pc2.header = point_cloud_data.header
    filtered_pc2.height = point_cloud_data.height
    filtered_pc2.width = point_cloud_data.width
    filtered_pc2.fields = point_cloud_data.fields
    filtered_pc2.is_bigendian = point_cloud_data.is_bigendian
    filtered_pc2.point_step = point_step
    filtered_pc2.row_step = point_cloud_data.row_step
    filtered_pc2.data = bytes(filtered_data)
    filtered_pc2.is_dense = False  

    return filtered_pc2


def get_color_from_point(point, is_bigendian):
    color_data = int(point[-1][-1])

    if is_bigendian:
        # Big endian
        r = (color_data >> 24) & 0xFF
        g = (color_data >> 16) & 0xFF
        b = (color_data >> 8) & 0xFF
    else:
        # Little endian
        r = color_data & 0xFF
        g = (color_data >> 8) & 0xFF
        b = (color_data >> 16) & 0xFF

    return (r, g, b)

def parse_points(data, point_step):
    points = []
    for i in range(0, len(data), point_step):
        point_data = data[i:i+point_step]
        x, y, z = struct.unpack_from('fff', point_data, 0)

        rgba = struct.unpack_from('I', point_data, 16)[0]
        r = (rgba >> 16) & 0xFF
        g = (rgba >> 8) & 0xFF
        b = rgba & 0xFF
        a = (rgba >> 24) & 0xFF 
        points.append((x, y, z, r, g, b, a))  
    return points

pc2_count = 0
should_continue = True
manager = None
subscriber = None
last_position = None
position_change_threshold = 0.0001
rotation_change_threshold = 0.0001
from tf.transformations import euler_from_quaternion

def get_robot_pose(tf_buffer):
    try:
        trans = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        position = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
        rotation = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
        return position, rotation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("TF lookup error: %s", e)
        return None, None

def check_robot_movement(start_pose, current_pose):
    start_position, start_rotation = start_pose
    current_position, current_rotation = current_pose
    
    if start_position and current_position:
        position_change = (abs(start_position[0] - current_position[0]) >= position_change_threshold or
                           abs(start_position[1] - current_position[1]) >= position_change_threshold or
                           abs(start_position[2] - current_position[2]) >= position_change_threshold)
    else:
        position_change = True
    
    if start_rotation and current_rotation:
        start_euler = euler_from_quaternion(start_rotation)
        current_euler = euler_from_quaternion(current_rotation)
        rotation_change = (abs(start_euler[0] - current_euler[0]) >= rotation_change_threshold or
                           abs(start_euler[1] - current_euler[1]) >= rotation_change_threshold or
                           abs(start_euler[2] - current_euler[2]) >= rotation_change_threshold)
        print(rotation_change)
    else:
        rotation_change = True
    
    return position_change or rotation_change

def callback(point_cloud_data):

    global pc2_count, should_continue, subscriber, manager

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    start_position = get_robot_pose(tf_buffer)
    if start_position is None:
        print("Failed to get robot position, skip this round...")
        return
    
    if should_continue and pc2_count < 1:
        if point_cloud_data.data:

            pc_arr = numpify(point_cloud_data)
            is_bigendian = point_cloud_data.is_bigendian
            point_step = point_cloud_data.point_step
            row_step = point_cloud_data.row_step
            id_dense = point_cloud_data.is_dense
            print("is_bigendian", is_bigendian, "point_step", point_step, "row_step", row_step, "id_dense", id_dense)
            data = point_cloud_data.data
            points = parse_points(data, point_step)
            color_to_filtered_points = {}
            color_to_filtered_points[(0, 0, 0)] = []

            for point in points:
                x, y, z, r, g, b, a = point
                color_key = (r, g, b)

                if color_key not in color_to_filtered_points:
                    color_to_filtered_points[color_key] = []
                color_to_filtered_points[color_key].append((x, y, z, r, g, b, a))

            for color, color_points in color_to_filtered_points.items():
                if color != (0, 0, 0):
                    print("got item", color[0])
                    filtered_pc2 = filter_point_cloud_by_color(point_cloud_data, (color[0], color[1], color[2]), point_step)
                    points = parse_points(point_cloud_data.data, point_step)

                    print(filtered_pc2.header.frame_id)
                    transed_filtered_pc2 = transform_point_cloud(filtered_pc2, 'map') # trans to a static frame

                    current_position = get_robot_pose(tf_buffer)
                    if current_position is None or check_robot_movement(start_position, current_position):
                        print("Robot moved more than 0.01m, current data is invalid.")
                        return
                    else:
                        pc2_count += 1
                        print("Successful {} pc".format(pc2_count))
                        manager.add_item(color[0], color[1], color[2], color_points, transed_filtered_pc2)
                        print("Done")

                else:
                    print("no det")

        else:
            print("NULL PC, skip...")
    else:
        print("Got enough data, stop...")
        should_continue = False
        subscriber.unregister()

def listener():
    global should_continue
    global subscriber
    global manager
    rospy.init_node('point_cloud_listener', anonymous=True)
    manager = ItemPCManager()
    manager.reset()

    subscriber = rospy.Subscriber('/head_camera/depth_seg/points', PointCloud2, callback)

    pc2_publisher = rospy.Publisher('/filtered_color_pc', PointCloud2, queue_size=10)

    print(subscriber)
    while True:
        if should_continue == False:
            print(manager.list_items())

            item_pc2 = manager.get_pc2_by_color(213, 212, 102) #!!!!!!!

            # table_pc2 = manager.get_pc2_by_color(0, 0, 0)

            '''
            if item_pc2:
                while True:
                    pc2_publisher.publish(item_pc2)
                    rospy.sleep(0.1)
            '''

            # pc2_msg is confirmed correct

            # Get cam trans
            tfBuffer = tf2_ros.Buffer()
            listener_tf = tf2_ros.TransformListener(tfBuffer)

            rate = rospy.Rate(10.0) 
            while not rospy.is_shutdown():
                try:
                    now = rospy.Time.now()
                    past = now - rospy.Duration(5.0) 
                    if tfBuffer.can_transform('base_link', 'base_link', now):
                        camera_trans = tfBuffer.lookup_transform('base_link', 'base_link', now) # Duplicated trans?

                        break
                    else:
                        rospy.loginfo("Waiting for transform")
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                    rospy.loginfo("Transform error: %s", ex)
                rate.sleep()

            if item_pc2 == []:
                print("item_pc2 empty")
                sys.exit()
            
            # Trans item_pc2 and table_pc2
            transed_item_pc2 = transform_point_cloud(item_pc2, 'base_link')
            # transed_table_pc2 = transform_point_cloud(table_pc2, 'base_link')

            '''
            if transed_item_pc2:
                while True:
                    pc2_publisher.publish(transed_item_pc2)
                    rospy.sleep(0.1)
            '''
            

            # Confirmed correct
            rospy.wait_for_service('grasp_predict')
            grasp_predictor = rospy.ServiceProxy('grasp_predict', Predict)

            try:
                predicted_grasp_result = grasp_predictor(transed_item_pc2, transed_item_pc2, camera_trans)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            publish_markers(predicted_grasp_result)

            if predicted_grasp_result.predicted_grasp_poses == []:
                print("no pred pose")
                sys.exit()
            
            
            # markers looks good

            moveit_commander.roscpp_initialize(sys.argv)
            move_group = moveit_commander.MoveGroupCommander("arm") 
            scene = moveit_commander.PlanningSceneInterface()
            move_group.set_max_velocity_scaling_factor(1.0)

            scene.clear()

            grasp_shift = np.array([[1,0,0,0.02],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            pre_grasp_shift = np.array([[1,0,0,-0.1],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            pick_shift = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.05],[0,0,0,1]])

            for i in range(len(predicted_grasp_result.predicted_grasp_poses)):
                move_group.set_start_state_to_current_state()
                
                # generate grasp pose
                grasp_pose = numpify(predicted_grasp_result.predicted_grasp_poses[i].pose).dot(grasp_shift)

                # calculate the pre-grasp pose
                pre_grasp_pose = grasp_pose.dot(pre_grasp_shift)
                
                # calculate the pick-up pose
                pick_up_pose = pick_shift.dot(grasp_pose)

                trans = tf.transformations.translation_from_matrix(pre_grasp_pose).tolist()
                quat = tf.transformations.quaternion_from_matrix(pre_grasp_pose).tolist()
                
                #trans = tf.transformations.translation_from_matrix(grasp_pose).tolist()
                #quat = tf.transformations.quaternion_from_matrix(grasp_pose).tolist()

                move_group.clear_pose_targets()
                move_group.set_pose_target(trans + quat)
                plan_result = move_group.plan()
                print(plan_result)
                if plan_result[0]:
                    joint_state = JointState()
                    joint_state.header.stamp = rospy.Time.now()
                    joint_state.name = plan_result[1].joint_trajectory.joint_names
                    joint_state.position = plan_result[1].joint_trajectory.points[-1].positions
                    moveit_robot_state = move_group.get_current_state()
                    moveit_robot_state.joint_state = joint_state
                    after_pre_grasp_current_config = moveit_robot_state
                    move_group.set_start_state(moveit_robot_state)
                    (approach_plan, fraction) = move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, grasp_pose)], 0.01, 0.0)
                    # check whether you can approach the object
                    if fraction < 0.9:
                        continue
                    
                    moveit_robot_state = RobotState()
                    joint_state.header.stamp = rospy.Time.now()
                    joint_state.position = approach_plan.joint_trajectory.points[-1].positions
                    moveit_robot_state.joint_state = joint_state
                    move_group.set_start_state(moveit_robot_state)
                    (pick_plan, fraction) = move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, pick_up_pose)], 0.01, 0.0)
                    # check whether you can pick the object
                    if fraction < 0.9:
                        continue
                        
                    print("got a way to pick up the object")
                    
                    break

            move_group.clear_pose_targets()
            scene.clear()

            # actual execution
            move_group.execute(plan_result[1])

            # apporach the object
            move_group.set_start_state_to_current_state()
            (approach_plan, fraction) = move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, grasp_pose)], 0.01, 0.0)
            move_group.execute(approach_plan)

            move_group.set_start_state_to_current_state()
            (pick_plan, fraction) = move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, pick_up_pose)], 0.01, 0.0)
            move_group.execute(pick_plan)

            rospy.sleep(1)
            sys.exit()

listener()
