import os
import rospy
from foliated_base_class import BaseIntersection, BaseTaskMotion, BaseVisualizer
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA
import moveit_msgs.msg
import trajectory_msgs.msg
from jiaming_helper import convert_joint_values_to_robot_state, make_mesh
from visualization_msgs.msg import Marker, MarkerArray
from ros_numpy import msgify, numpify
import numpy as np
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
import random

class ManipulationTaskMotion(BaseTaskMotion):
    def __init__(self, planned_motion, has_object_in_hand, object_pose, object_mesh_path, obstacle_pose, obstacle_mesh_path):
        # if planned_motion must be trajectory_msgs/JointTrajectory.
        if not isinstance(planned_motion, moveit_msgs.msg.RobotTrajectory):
            raise TypeError("planned_motion must be trajectory_msgs/JointTrajectory.")

        self.planned_motion = planned_motion
        self.has_object_in_hand = has_object_in_hand # if the object is in hand.
        self.object_pose = object_pose # if the object is in hand, then this is the object pose in the hand frame. if not, this is the object pose in the base_link frame.
        self.object_mesh_path = object_mesh_path
        self.obstacle_pose = obstacle_pose
        self.obstacle_mesh_path = obstacle_mesh_path

    def get(self):
        return self.planned_motion, self.has_object_in_hand, self.object_pose, self.object_mesh_path, self.obstacle_pose, self.obstacle_mesh_path


class MoveitVisualizer(BaseVisualizer):

    def prepare_visualizer(self, active_joints, robot):
        '''
        This function prepares the visualizer.
        '''
        rospy.wait_for_service('/compute_fk')
        self.compute_fk_srv = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        # initialize a ros marker array publisher
        self.sampled_robot_state_publisher = rospy.Publisher(
            "sampled_robot_state",
            MarkerArray,
            queue_size=5)

        self.sampled_marker_ids = []

        self.distribution_robot_state_publisher = rospy.Publisher(
            "/move_group/result_distribution_robot_state",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=5,
        )

        # self.object_publisher = rospy.Publisher('/intermediate_object', Marker, queue_size=10)
        self.object_publisher = rospy.Publisher('/intermediate_object', MarkerArray, queue_size=10)
        # this is used to display the planned path in rviz
        self.display_robot_state_publisher = rospy.Publisher(
            "/move_group/result_display_robot_state",
            moveit_msgs.msg.DisplayRobotState,
            queue_size=5,
        )
        self.active_joints = active_joints
        self.robot = robot

        self.whole_scene_marker_array = MarkerArray()

        self.manipulated_object_marker = Marker()
        self.manipulated_object_marker.header.frame_id = "base_link"
        self.manipulated_object_marker.id = 0
        self.manipulated_object_marker.type = Marker.MESH_RESOURCE
        self.manipulated_object_marker.scale = Point(1,1,1)
        self.manipulated_object_marker.color = ColorRGBA(0,1,0,1)
        # manipulated_object_marker.mesh_resource = "package://task_planner/mesh_dir/" + os.path.basename(task_planner.manifold_info[(experiment.start_foliation_id, experiment.start_manifold_id)].object_mesh)
        self.whole_scene_marker_array.markers.append(self.manipulated_object_marker)

        self.obstacle_marker = Marker()
        self.obstacle_marker.header.frame_id = "base_link"
        self.obstacle_marker.id = 1
        self.obstacle_marker.type = Marker.MESH_RESOURCE
        self.obstacle_marker.scale = Point(1,1,1)
        self.obstacle_marker.color = ColorRGBA(1,1,1,1)
        self.whole_scene_marker_array.markers.append(self.obstacle_marker)
        
        self.current_object_pose_stamped = PoseStamped()
        self.current_object_pose_stamped.header.frame_id = "wrist_roll_link"
        self.current_object_pose_stamped.pose = Pose()

        self.attached_object = moveit_msgs.msg.AttachedCollisionObject()
        self.attached_object.link_name = "wrist_roll_link"
        self.attached_object.touch_links = ["l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"]

    def visualize_sampled_configurations(self, sampled_configurations):
        # use the moveit visualizer to visualize the sampled configurations
        # sampled_configurations is a list of configurations

        # if number of sampled configuration is more than 100, then only visualize 100 of them randomly
        if len(sampled_configurations) > 100:
            sampled_configurations = random.sample(sampled_configurations, 100)

        delete_marker_arrary = MarkerArray()
        for i in self.sampled_marker_ids:
            delete_marker = Marker()
            delete_marker.header.frame_id = "base_link"
            delete_marker.id = i
            delete_marker.action = Marker.DELETE
            delete_marker_arrary.markers.append(delete_marker)
        self.sampled_robot_state_publisher.publish(delete_marker_arrary)

        # create a marker array
        marker_array = MarkerArray()

        for t, c in enumerate(sampled_configurations):
            # convert the sampled configuration into RobotState
            current_robot_state = convert_joint_values_to_robot_state(c, self.active_joints, self.robot)

            # pass current robot state to compute fk service
            fk_request = GetPositionFKRequest()
            fk_request.header.frame_id = "base_link"
            fk_request.fk_link_names = self.robot.get_link_names(group="arm")
            fk_request.robot_state = current_robot_state

            fk_response = self.compute_fk_srv(fk_request)

            arm_marker = Marker()
            arm_marker.header.frame_id = "base_link"
            arm_marker.id = 3*t
            arm_marker.type = Marker.LINE_STRIP
            arm_marker.scale = Point(0.02, 0.02, 0.02)
            arm_marker.color = ColorRGBA(1,0,0,1)
            arm_marker.points = [Point(p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in fk_response.pose_stamped]
            marker_array.markers.append(arm_marker)
            self.sampled_marker_ids.append(3*t)

            # fk_request.fk_link_names = self.robot.get_link_names(group="gripper")
            # fk_response = self.compute_fk_srv(fk_request)

            l_finger_marker = Marker()
            l_finger_marker.header.frame_id = "base_link"
            l_finger_marker.id = 3*t + 1
            l_finger_marker.type = Marker.CUBE
            l_finger_marker.scale = Point(0.1, 0.02, 0.02)
            l_finger_marker.color = ColorRGBA(0,1,0,1)
            l_finger_marker.pose = msgify(Pose, np.dot(numpify(fk_response.pose_stamped[-1].pose), np.array([[1,0,0,0.15],[0,1,0,0.045],[0,0,1,0],[0,0,0,1]])))
            marker_array.markers.append(l_finger_marker)
            self.sampled_marker_ids.append(3*t + 1)

            r_finger_marker = Marker()
            r_finger_marker.header.frame_id = "base_link"
            r_finger_marker.id = 3*t + 2
            r_finger_marker.type = Marker.CUBE
            r_finger_marker.scale = Point(0.1, 0.02, 0.02)
            r_finger_marker.color = ColorRGBA(0,1,0,1)
            r_finger_marker.pose = msgify(Pose, np.dot(numpify(fk_response.pose_stamped[-1].pose), np.array([[1,0,0,0.15],[0,1,0,-0.045],[0,0,1,0],[0,0,0,1]])))
            marker_array.markers.append(r_finger_marker)
            self.sampled_marker_ids.append(3*t + 2)

        self.sampled_robot_state_publisher.publish(marker_array)
        # wait for the marker to be published
        rospy.sleep(1.0)


    def visualize_distribution(self, distributions):
        '''
        distribution is a list of distribution.
        '''
        if len(distributions) > 0:

            display_trajectory_msg = moveit_msgs.msg.DisplayTrajectory()
            # display_trajectory_msg.model_id = self.robot.get_name()
            display_trajectory_msg.trajectory_start = convert_joint_values_to_robot_state(distributions[0].mean, self.active_joints, self.robot)

            robot_trajectory_msg = moveit_msgs.msg.RobotTrajectory()
            robot_trajectory_msg.joint_trajectory.joint_names = self.active_joints
            for t, d in enumerate(distributions):
                joint_trajectory_point = trajectory_msgs.msg.JointTrajectoryPoint()
                joint_trajectory_point.positions = d.mean.tolist()
                joint_trajectory_point.time_from_start = rospy.Duration(0.02 * t)
                robot_trajectory_msg.joint_trajectory.points.append(joint_trajectory_point)
            # robot_trajectory_msg.joint_trajectory.points
            # [d.mean.tolist() for d in distributions]

            display_trajectory_msg.trajectory.append(robot_trajectory_msg)

            self.distribution_robot_state_publisher.publish(display_trajectory_msg)

    def visualize_plan(self, list_of_motion_plan):
        '''
        This function will receive a list of motion plan and visualize it.
        One thing must be considered is that this list contains both motion between intersections and
        intersection. Therefore, the visuliaztion must handle this situation properly.
        '''
        print "visualize the plan"
        print "press ctrl+c to exit"
        
        need_to_break = False

        while not rospy.is_shutdown():
            for motion_plan in list_of_motion_plan:
                motion_trajectory, has_object_in_hand, object_pose, object_mesh_path, obstacle_pose, obstacle_mesh_path = motion_plan.get()

                if obstacle_pose is not None:
                    self.obstacle_marker.mesh_resource = "package://task_planner/mesh_dir/" + os.path.basename(obstacle_mesh_path)
                    self.obstacle_marker.action = Marker.ADD
                    self.obstacle_marker.pose = msgify(Pose, obstacle_pose)

                for p in motion_trajectory.joint_trajectory.points:
                    current_robot_state_msg = moveit_msgs.msg.DisplayRobotState()
                    current_robot_state_msg.state = convert_joint_values_to_robot_state(p.positions, self.active_joints, self.robot)

                    # if not manipulated object, then does not need to publish the object
                    if object_pose is not None:
                        if has_object_in_hand:
                            self.attached_object.object = make_mesh(
                                "object", 
                                self.current_object_pose_stamped, 
                                object_mesh_path
                            )
                            self.attached_object.object.pose = msgify(Pose, np.linalg.inv(object_pose))
                            current_robot_state_msg.state.attached_collision_objects.append(self.attached_object)
                            self.manipulated_object_marker.action = Marker.DELETE
                        else:
                            self.manipulated_object_marker.mesh_resource = "package://task_planner/mesh_dir/" + os.path.basename(object_mesh_path)
                            self.manipulated_object_marker.action = Marker.ADD
                            self.manipulated_object_marker.pose = msgify(Pose, object_pose)

                        self.object_publisher.publish(self.whole_scene_marker_array)

                    self.display_robot_state_publisher.publish(current_robot_state_msg)

                    # rospy sleep
                    rospy.sleep(0.03)

                    if rospy.is_shutdown():
                        need_to_break = True
                        break
                if need_to_break:
                    break
            if need_to_break:
                break

