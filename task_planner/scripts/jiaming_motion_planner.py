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

class BaseMotionPlanner(object):

    def prepare_planner(self):
        # Prepares the planner
        raise NotImplementedError("Please Implement this method")

    def plan(self, start_configuration, goal_configuration, constraints, planning_hint):
        # Returns a motion plan which can be visualized.
        raise NotImplementedError("Please Implement this method")

    def visualize_plan(self, plan):
        # Visualizes the plan
        raise NotImplementedError("Please Implement this method")

    def shutdown_planner(self):
        # Deletes the planner
        raise NotImplementedError("Please Implement this method")

class MoveitMotionPlanner(BaseMotionPlanner):
    def prepare_planner(self):
        rospy.init_node('main_pipeline_node', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(0.5) # wait for the planning scene to be ready
        scene.clear()

        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_group.set_planner_id('CDISTRIBUTIONRRTConfigDefault')
        self.move_group.set_planning_time(2.0)

        # this is used to display the planned path in rviz
        self.display_robot_state_publisher = rospy.Publisher(
            "/move_group/result_display_robot_state",
            moveit_msgs.msg.DisplayRobotState,
            queue_size=5,
        )

    def plan(self, start_configuration, goal_configuration, constraints, planning_hint):
        # reset the motion planner
        self.move_group.clear_path_constraints()
        self.move_group.clear_in_hand_pose()

        start_moveit_robot_state = convert_joint_values_to_robot_state(start_configuration, self.move_group.get_active_joints(), self.robot)

        # if you have object in hand, then you need to set the object in hand pose
        

        # set the start configuration
        self.move_group.set_start_state(start_moveit_robot_state)
        self.move_group.set_joint_value_target(goal_configuration)

        motion_plan_result = self.move_group.plan()

        return success_flag, motion_plan_result

    def shutdown_planner(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)