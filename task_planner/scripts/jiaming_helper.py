import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, PositionConstraint, BoundingVolume, MoveItErrorCodes
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Point32
import tf.transformations as tf_trans
from shape_msgs.msg import SolidPrimitive

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

def construct_moveit_constraint(in_hand_pose_, constraint_pose_, orientation_constraint_, position_constraint_):

    moveit_quaternion = tf_trans.quaternion_from_matrix(constraint_pose_) # return x, y z, w
    moveit_translation = tf_trans.translation_from_matrix(constraint_pose_) # return x, y, z

    moveit_constraint = Constraints()
    moveit_constraint.name = "use_equality_constraints"

    oc = OrientationConstraint()

    oc.parameterization = OrientationConstraint.ROTATION_VECTOR
    oc.header.frame_id = "base_link"
    oc.header.stamp = rospy.Time(0)
    oc.link_name = "wrist_roll_link"
    constrained_quaternion = Quaternion()
    constrained_quaternion.x = moveit_quaternion[0]
    constrained_quaternion.y = moveit_quaternion[1]
    constrained_quaternion.z = moveit_quaternion[2]
    constrained_quaternion.w = moveit_quaternion[3]
    oc.orientation = constrained_quaternion
    oc.weight = 1.0

    oc.absolute_x_axis_tolerance = orientation_constraint_[0] * 2.0
    oc.absolute_y_axis_tolerance = orientation_constraint_[1] * 2.0
    oc.absolute_z_axis_tolerance = orientation_constraint_[2] * 2.0

    moveit_constraint.orientation_constraints.append(oc)

    pc = PositionConstraint()
    pc.header.frame_id = "base_link"
    pc.header.stamp = rospy.Time(0)
    pc.link_name = "wrist_roll_link"
    pc.target_point_offset.x = moveit_translation[0]
    pc.target_point_offset.y = moveit_translation[1]
    pc.target_point_offset.z = moveit_translation[2]

    pc.target_point_offset.x = 0.0
    pc.target_point_offset.y = 0.0
    pc.target_point_offset.z = 0.0

    pc.weight = 1.0

    solid_primitive = SolidPrimitive()
    solid_primitive.type = SolidPrimitive.BOX
    solid_primitive.dimensions = [position_constraint_[0], position_constraint_[1], position_constraint_[2]]

    bounding_volume = BoundingVolume()
    bounding_volume.primitives.append(solid_primitive)

    bounding_volume_pose = Pose()
    bounding_volume_pose.position.x = moveit_translation[0]
    bounding_volume_pose.position.y = moveit_translation[1]
    bounding_volume_pose.position.z = moveit_translation[2]

    bounding_volume_pose.orientation.x = moveit_quaternion[0]
    bounding_volume_pose.orientation.y = moveit_quaternion[1]
    bounding_volume_pose.orientation.z = moveit_quaternion[2]
    bounding_volume_pose.orientation.w = moveit_quaternion[3]

    bounding_volume.primitive_poses.append(bounding_volume_pose)

    pc.constraint_region = bounding_volume

    moveit_constraint.position_constraints.append(pc)

    # convert in_hand_pose_ from matrix to Pose
    in_hand_quaternion = tf_trans.quaternion_from_matrix(in_hand_pose_) # return x, y z, w
    in_hand_translation = tf_trans.translation_from_matrix(in_hand_pose_) # return x, y, z

    in_hand_pose = Pose()
    in_hand_pose.position.x = in_hand_translation[0]
    in_hand_pose.position.y = in_hand_translation[1]
    in_hand_pose.position.z = in_hand_translation[2]

    in_hand_pose.orientation.x = in_hand_quaternion[0]
    in_hand_pose.orientation.y = in_hand_quaternion[1]
    in_hand_pose.orientation.z = in_hand_quaternion[2]
    in_hand_pose.orientation.w = in_hand_quaternion[3]

    moveit_constraint.in_hand_pose = in_hand_pose

    return moveit_constraint