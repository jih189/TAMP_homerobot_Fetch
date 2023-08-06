#!/usr/bin/env python
from experiment_helper import Experiment, Manifold, Intersection
from jiaming_task_planner import MTGTaskPlanner, MDPTaskPlanner, MTGTaskPlannerWithGMM, MDPTaskPlannerWithGMM, GMM, ManifoldDetail, IntersectionDetail
from jiaming_helper import convert_joint_values_to_robot_trajectory, convert_joint_values_to_robot_state, get_no_constraint

import sys
import copy
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetJointWithConstraints, GetJointWithConstraintsRequest
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, MoveItErrorCodes
from sensor_msgs.msg import JointState
import tf.transformations as tf_trans
from ros_numpy import numpify, msgify
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Point32
import trimesh
from trimesh import transformations
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
import struct
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

np.set_printoptions(suppress=True, precision = 3)
if __name__ == "__main__":

    ##########################################################
    #################### experiment setup ####################
    max_attempt_times = 1

    experiment_name = "pick_and_place"
    # experiment_name = "move_mouse"

    use_mtg = True # use mtg or mdp
    use_gmm = True # use gmm or not

    ##########################################################

    rospack = rospkg.RosPack()
    # Get the path of the desired package
    package_path = rospack.get_path('task_planner')

    # load the gmm
    gmm_dir_path = package_path + '/gmm/'
    gmm = GMM()
    gmm.load_distributions(gmm_dir_path)

    # load the expierment
    experiment = Experiment()
    experiment.load(package_path + "/experiment_dir/" + experiment_name)

    # load the experiment into the task planner
    if use_mtg:
        if use_gmm:
            task_planner = MTGTaskPlannerWithGMM(gmm)
        else:
            task_planner = MTGTaskPlanner()
    else:
        if use_gmm:
            task_planner = MDPTaskPlannerWithGMM(gmm)
        else:
            task_planner = MDPTaskPlanner()



    task_planner.reset_task_planner()

    #############################################################################
    # setup the task graph.
    # add manifolds
    for manifold in experiment.manifolds:
        # the manifold id in the task planner should be a pair of (foliation_id, manifold_id)
        # the constraint here has (moveit constraint, has_object_in_hand, object_pose, object mesh, object name)
        # if the has_object_in_hand is true, then object pose here is the in-hand-object-pose which is the grasp pose in the object frame
        # if the has_object_in_hand is false, then object pose here is the object placement pose which is the placement pose in the world frame
        
        # initialize the moveit constraint.
        no_constraint = get_no_constraint()

        # if the manifold has object in hand, then the object pose is the grasp pose in the object frame
        # if the manifold has no object in hand, then the object pose is the placement pose in the world frame
        manifold_object_pose = manifold.in_hand_pose if manifold.has_object_in_hand else manifold.object_pose

        # manifold.has_object_in_hand
        task_planner.add_manifold(
            ManifoldDetail(
                no_constraint, 
                manifold.has_object_in_hand, 
                manifold_object_pose, 
                manifold.object_mesh, 
                manifold.object_name
            ), 
            (manifold.foliation_id, manifold.manifold_id)
        )

    # add intersections
    for intersection in experiment.intersections:
        task_planner.add_intersection(
            (intersection.foliation_id_1, intersection.manifold_id_1),
            (intersection.foliation_id_2, intersection.manifold_id_2),
            IntersectionDetail(
                intersection.has_object_in_hand, 
                intersection.trajectory_motion, 
                intersection.in_hand_pose,
                intersection.object_mesh,
                intersection.object_name
            )
        )

    #############################################################################
    # initialize the motion planner and planning scene of moveit
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('main_pipeline_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(0.5) # wait for the planning scene to be ready
    scene.clear()
    move_group = moveit_commander.MoveGroupCommander("arm")

    move_group.set_planner_id('CDISTRIBUTIONRRTConfigDefault')
    # if use_gmm:
    #     move_group.set_planner_id('CDISTRIBUTIONRRTConfigDefault')
    # else:
    #     move_group.set_planner_id('CBIRRTConfigDefault')

    display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,
        )

    # set initial joint state
    joint_state_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)

    # Create a JointState message
    initial_joint_state = JointState()
    initial_joint_state.header.stamp = rospy.Time.now()
    initial_joint_state.name = ['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'wrist_flex_joint', 'l_gripper_finger_joint', 'r_gripper_finger_joint']
    initial_joint_state.position = [0.38, -1.28, 1.51, 0.35, 1.81, 1.47, 0.04, 0.04]

    rate = rospy.Rate(10)
    while(joint_state_publisher.get_num_connections() < 1): # need to wait until the publisher is ready.
        rate.sleep()
    joint_state_publisher.publish(initial_joint_state)

    ##############################################################################

    # load the obstacle into the planning scene.
    obstacle_pose_stamped = PoseStamped()
    obstacle_pose_stamped.header.frame_id = "base_link"
    obstacle_pose_stamped.pose = msgify(geometry_msgs.msg.Pose, experiment.obstacle_mesh_pose)
    scene.add_mesh("obstacle", obstacle_pose_stamped, experiment.obstacle_mesh, size=(1,1,1))

    ##############################################################################################

    # set start and goal configurations with relative foliation and manifold id
    task_planner.set_start_and_goal(
        (experiment.start_foliation_id, experiment.start_manifold_id), # start manifold id
        move_group.get_current_joint_values(), # start configuration
        (experiment.goal_foliation_id, experiment.goal_manifold_id), # goal manifold id
        move_group.get_current_joint_values() # goal configuration
    )

    ##############################################################################
    # start the main pipeline

    for _ in range(max_attempt_times):
        # generate task sequence
        task_sequence = task_planner.generate_task_sequence()
        if len(task_sequence) == 0: # if no task sequence found, then break the loop
            print("no task sequence found")
            break

        found_solution = True
        solution_path = []

        # motion planner tries to solve each task in the task sequence
        for task in task_sequence:
            # print the task detail here
            # task.print_task_detail()

            if task.manifold_detail.has_object_in_hand: # has object in hand
                
                # attach the object to the hand in the planning scene
                target_object_pose = PoseStamped()
                target_object_pose.header.frame_id = "base_link"
                target_object_pose.pose = msgify(geometry_msgs.msg.Pose, numpify(move_group.get_current_pose().pose).dot(np.linalg.inv(task.manifold_detail.object_pose)))
                scene.attach_mesh(
                    "wrist_roll_link", # link
                    task.manifold_detail.object_name, # name
                    target_object_pose, # pose
                    task.manifold_detail.object_mesh, # filename
                    size=(1,1,1), #size
                    touch_links=["l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"] #touch_links
                )

                # check whether the attached object is in the planning scene
                # if it is not, wait for short time.
                while task.manifold_detail.object_name not in scene.get_attached_objects():
                    rospy.sleep(0.0001)
                
                # do the motion planning
                move_group.clear_path_constraints()
                
                # set start and goal congfiguration to motion planner.
                start_moveit_robot_state = convert_joint_values_to_robot_state(task.start_configuration, move_group.get_active_joints(), robot)
                move_group.set_start_state(start_moveit_robot_state)
                move_group.set_joint_value_target(task.goal_configuration)
                move_group.set_path_constraints(task.manifold_detail.constraint)

                motion_plan_result = move_group.plan()

                # if the planner uses gmm, then it will convert sampled data in the robot state format.
                # so we need to convert it back to the numpy format based on the active joints.
                if(use_gmm):
                    for m in motion_plan_result[4].verified_motions:
                        m.sampled_state = [m.sampled_state.joint_state.position[m.sampled_state.joint_state.name.index(j)] for j in move_group.get_active_joints()]

                task_planner.update(task.task_graph_info, motion_plan_result)

                if not motion_plan_result[0]: #if the motion planner can't find a solution, then replan
                    found_solution = False
                    break

                solution_path.append(motion_plan_result[1])

                # add the intersection motion to the solution path
                if(len(task.next_motion) > 1):
                    intersection_motion = convert_joint_values_to_robot_trajectory(task.next_motion, move_group.get_active_joints())
                    solution_path.append(intersection_motion)

                # remove the attached object from the planning scene
                scene.remove_attached_object("wrist_roll_link", task.manifold_detail.object_name)

                # check whether the attached object is in the planning scene
                # if it is, wait for short time.
                while task.manifold_detail.object_name in scene.get_attached_objects():
                    rospy.sleep(0.0001)

            else:
                # add the object to the planning scene
                target_object_pose = PoseStamped()
                target_object_pose.header.frame_id = "base_link"
                target_object_pose.pose = msgify(geometry_msgs.msg.Pose, task.manifold_detail.object_pose)
                scene.add_mesh(task.manifold_detail.object_name, target_object_pose, task.manifold_detail.object_mesh, size=(1,1,1))
    
                # check whether the object is in the planning scene
                # if it is not, wait for short time.
                while task.manifold_detail.object_name not in scene.get_known_object_names():
                    rospy.sleep(0.0001)

                # do the motion planning
                move_group.clear_path_constraints()
                
                # set start and goal congfiguration to motion planner.
                start_moveit_robot_state = convert_joint_values_to_robot_state(task.start_configuration, move_group.get_active_joints(), robot)
                move_group.set_start_state(start_moveit_robot_state)
                move_group.set_joint_value_target(task.goal_configuration)
                move_group.set_path_constraints(task.manifold_detail.constraint)

                motion_plan_result = move_group.plan()

                # if the planner uses gmm, then it will convert sampled data in the robot state format.
                # so we need to convert it back to the numpy format based on the active joints.
                if(use_gmm): 
                    for m in motion_plan_result[4].verified_motions:
                        m.sampled_state = [m.sampled_state.joint_state.position[m.sampled_state.joint_state.name.index(j)] for j in move_group.get_active_joints()]

                task_planner.update(task.task_graph_info, motion_plan_result)

                if not motion_plan_result[0]: #if the motion planner can't find a solution, then replan
                    found_solution = False
                    break

                solution_path.append(motion_plan_result[1])

                # add the intersection motion to the solution path
                if(len(task.next_motion) > 1):
                    intersection_motion = convert_joint_values_to_robot_trajectory(task.next_motion, move_group.get_active_joints())
                    solution_path.append(intersection_motion)

                # remove the object from the planning scene
                scene.remove_world_object(task.manifold_detail.object_name)

                # check whether the object is in the planning scene
                # if it is, wait for short time.
                while task.manifold_detail.object_name in scene.get_known_object_names():
                    rospy.sleep(0.0001)

        if found_solution: # found solution, then break the

            print("found solution")
            # try to execute the solution path

            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = move_group.get_current_state()
            display_trajectory.trajectory = solution_path

            display_trajectory_publisher.publish(display_trajectory)
            
            break