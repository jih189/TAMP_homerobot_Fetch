import rospy
from foliated_base_class import BaseIntersection, BaseTaskMotion, BaseVisualizer
import moveit_msgs.msg
from jiaming_helper import convert_joint_values_to_robot_state
from visualization_msgs.msg import Marker, MarkerArray

class ManipulationTaskMotion(BaseTaskMotion):
    def __init__(self, planned_motion, has_object_in_hand, object_pose):
        # if planned_motion must be trajectory_msgs/JointTrajectory.
        if not isinstance(planned_motion, moveit_msgs.msg.RobotTrajectory):
            raise TypeError("planned_motion must be trajectory_msgs/JointTrajectory.")

        self.planned_motion = planned_motion
        self.has_object_in_hand = has_object_in_hand # if the object is in hand.
        self.object_pose = object_pose # if the object is in hand, then this is the object pose in the hand frame. if not, this is the object pose in the base_link frame.

    def get(self):
        return self.planned_motion, self.has_object_in_hand, self.object_pose


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

        manipulated_object_marker = Marker()
        manipulated_object_marker.header.frame_id = "base_link"
        manipulated_object_marker.id = 0
        manipulated_object_marker.type = Marker.MESH_RESOURCE
        manipulated_object_marker.scale = Point(1,1,1)
        manipulated_object_marker.color = ColorRGBA(0,1,0,1)
        # manipulated_object_marker.mesh_resource = "package://task_planner/mesh_dir/" + os.path.basename(task_planner.manifold_info[(experiment.start_foliation_id, experiment.start_manifold_id)].object_mesh)

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
                motion_trajectory, has_object_in_hand, object_pose = motion_plan.get()

                for p in motion_trajectory.joint_trajectory.points:
                    current_robot_state_msg = moveit_msgs.msg.DisplayRobotState()
                    current_robot_state_msg.state = convert_joint_values_to_robot_state(p.positions, self.active_joints, self.robot)

                    # if has_object_in_hand:
                        
                    # else:

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

