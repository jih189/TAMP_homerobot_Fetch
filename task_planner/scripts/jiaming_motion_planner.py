import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetJointWithConstraints, GetJointWithConstraintsRequest
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, MoveItErrorCodes, AttachedCollisionObject
from sensor_msgs.msg import JointState
from ros_numpy import numpify, msgify
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Point32

from jiaming_helper import convert_joint_values_to_robot_trajectory, convert_joint_values_to_robot_state, get_no_constraint, construct_moveit_constraint, make_mesh 
from foliated_base_class import BaseMotionPlanner
from jiaming_visualizer import ManipulationTaskMotion

class MoveitMotionPlanner(BaseMotionPlanner):
    def prepare_planner(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(0.5) # wait for the planning scene to be ready
        self.scene.clear()

        self.move_group = moveit_commander.MoveGroupCommander("arm")
        # self.move_group.set_planner_id('CDISTRIBUTIONRRTConfigDefault')
        self.move_group.set_planner_id('RRTConnectkConfigDefault')
        self.move_group.set_planning_time(2.0)

    def plan(self, start_configuration, goal_configuration, foliation_constraints, co_parameter, planning_hint):
        # reset the motion planner
        self.scene.clear()
        self.move_group.clear_path_constraints()
        self.move_group.clear_in_hand_pose()

        obstacle_pose_stamped = PoseStamped()
        obstacle_pose_stamped.header.frame_id = "base_link"
        obstacle_pose_stamped.pose = msgify(Pose, foliation_constraints['obstacle_pose'])

        # add the obstacle into the planning scene.
        self.scene.add_mesh("obstacle", obstacle_pose_stamped, foliation_constraints['obstacle_mesh'], size=(1,1,1))

        start_moveit_robot_state = convert_joint_values_to_robot_state(start_configuration, self.move_group.get_active_joints(), self.robot)

        # if you have object in hand, then you need to set the object in hand pose
        

        # set the start configuration
        self.move_group.set_start_state(start_moveit_robot_state)
        self.move_group.set_joint_value_target(goal_configuration)

        motion_plan_result = self.move_group.plan()

        # the section returned value should be a BaseTaskMotion
        return motion_plan_result[0], ManipulationTaskMotion(
                planned_motion=motion_plan_result[1], 
                has_object_in_hand=foliation_constraints['is_object_in_hand'], 
                object_pose=co_parameter,
                object_mesh_path=foliation_constraints['object_mesh_path'],
                obstacle_pose=foliation_constraints['obstacle_pose'],
                obstacle_mesh_path=foliation_constraints['obstacle_mesh']
            ), motion_plan_result

    def shutdown_planner(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)