import os
import rospy
from foliated_base_class import BaseIntersection, BaseTaskMotion, BaseVisualizer
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA
import moveit_msgs.msg
from jiaming_helper import convert_joint_values_to_robot_state, make_mesh
from visualization_msgs.msg import Marker, MarkerArray
from ros_numpy import msgify
import numpy as np

class ManipulationTaskMotion(BaseTaskMotion):
    def __init__(self, planned_motion, has_object_in_hand, object_pose, object_mesh_path):
        # if planned_motion must be trajectory_msgs/JointTrajectory.
        if not isinstance(planned_motion, moveit_msgs.msg.RobotTrajectory):
            raise TypeError("planned_motion must be trajectory_msgs/JointTrajectory.")

        self.planned_motion = planned_motion
        self.has_object_in_hand = has_object_in_hand # if the object is in hand.
        self.object_pose = object_pose # if the object is in hand, then this is the object pose in the hand frame. if not, this is the object pose in the base_link frame.
        self.object_mesh_path = object_mesh_path

    def get(self):
        return self.planned_motion, self.has_object_in_hand, self.object_pose, self.object_mesh_path


class MoveitVisualizer(BaseVisualizer):

    def prepare_visualizer(self, active_joints, robot):
        '''
        This function prepares the visualizer.
        '''
        self.object_publisher = rospy.Publisher('/intermediate_object', Marker, queue_size=10)
        # this is used to display the planned path in rviz
        self.display_robot_state_publisher = rospy.Publisher(
            "/move_group/result_display_robot_state",
            moveit_msgs.msg.DisplayRobotState,
            queue_size=5,
        )
        self.active_joints = active_joints
        self.robot = robot

        self.manipulated_object_marker = Marker()
        self.manipulated_object_marker.header.frame_id = "base_link"
        self.manipulated_object_marker.id = 0
        self.manipulated_object_marker.type = Marker.MESH_RESOURCE
        self.manipulated_object_marker.scale = Point(1,1,1)
        self.manipulated_object_marker.color = ColorRGBA(0,1,0,1)
        # manipulated_object_marker.mesh_resource = "package://task_planner/mesh_dir/" + os.path.basename(task_planner.manifold_info[(experiment.start_foliation_id, experiment.start_manifold_id)].object_mesh)

        self.current_object_pose_stamped = PoseStamped()
        self.current_object_pose_stamped.header.frame_id = "wrist_roll_link"
        self.current_object_pose_stamped.pose = Pose()

        self.attached_object = moveit_msgs.msg.AttachedCollisionObject()
        self.attached_object.link_name = "wrist_roll_link"
        self.attached_object.touch_links = ["l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"]


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
                motion_trajectory, has_object_in_hand, object_pose, object_mesh_path = motion_plan.get()

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

                        self.object_publisher.publish(self.manipulated_object_marker)

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

