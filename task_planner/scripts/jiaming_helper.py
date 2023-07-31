import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, MoveItErrorCodes
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Point32

# convert a list of joint values to robotTrajectory
def convert_joint_values_to_robot_trajectory(joint_values_list_, joint_names_):
    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory = JointTrajectory()
    robot_trajectory.joint_trajectory.joint_names = joint_names_

    for i in range(len(joint_values_list_)):
        robot_trajectory.joint_trajectory.points.append(JointTrajectoryPoint())
        robot_trajectory.joint_trajectory.points[i].positions = joint_values_list_[i]
        robot_trajectory.joint_trajectory.points[i].velocities = [0.0] * len(joint_values_list_[i])
        robot_trajectory.joint_trajectory.points[i].accelerations = [0.0] * len(joint_values_list_[i])
        robot_trajectory.joint_trajectory.points[i].time_from_start = rospy.Duration(0.1 * i)

    return robot_trajectory

def convert_joint_values_to_robot_state(joint_values_list_, joint_names_, robot_):
    '''
    convert a list of joint values to robotState
    joint_values_list_: a list of joint values
    joint_names_: a list of joint names
    robot_: a robotCommander
    '''
    moveit_robot_state = robot_.get_current_state()
    position_list = list(moveit_robot_state.joint_state.position)
    for joint_name, joint_value in zip(joint_names_, joint_values_list_):
        position_list[moveit_robot_state.joint_state.name.index(joint_name)] = joint_value
    moveit_robot_state.joint_state.position = tuple(position_list)
    return moveit_robot_state

def get_no_constraint():
    no_constraint = Constraints()
    no_constraint.name = "use_equality_constraints"

    oc = OrientationConstraint()

    oc.parameterization = OrientationConstraint.ROTATION_VECTOR
    oc.header.frame_id = "base_link"
    oc.header.stamp = rospy.Time(0)
    oc.link_name = "wrist_roll_link"
    constrained_quaternion = Quaternion()
    constrained_quaternion.x = 0.0
    constrained_quaternion.y = 0.0
    constrained_quaternion.z = 0.0
    constrained_quaternion.w = 1.0
    oc.orientation = constrained_quaternion
    oc.weight = 1.0

    oc.absolute_x_axis_tolerance = 2 * 3.1415
    oc.absolute_y_axis_tolerance = 2 * 3.1415
    oc.absolute_z_axis_tolerance = 2 * 3.1415
    no_constraint.orientation_constraints.append(oc)

    # need to set in-hand pose
    in_hand_pose = Pose()
    in_hand_pose.position.x = 0.0
    in_hand_pose.position.y = 0.0
    in_hand_pose.position.z = 0.0
    in_hand_pose.orientation.x = 0.0
    in_hand_pose.orientation.y = 0.0
    in_hand_pose.orientation.z = 0.0
    in_hand_pose.orientation.w = 1.0
    no_constraint.in_hand_pose = in_hand_pose
    return no_constraint