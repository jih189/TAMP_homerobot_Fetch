
'''
This script acts as an experiment monitor, it runs the main pipeline and monitors the output of the pipeline.
'''

import datetime
import os
import sys
import subprocess
import pickle
import rospy
import numpy as np
print(sys.version)
import time

import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

# add the coppeliasim remote api path
sys.path.append(os.path.expanduser('~') + "/CoppeliaSim/programming/zmqRemoteApi/clients/python/")
print(sys.path)

# coppeliasim imports
from zmqRemoteApi import RemoteAPIClient

WS_BASE = os.path.expanduser('~') +  "/catkin_ws"

def get_object_pose(object_name):
    # because tf doesn't play nice with python3, we have to directly call the tf_echo command
    # the tf_echo command returns a string with the pose of the object, we only want the translation
    tf_out = subprocess.Popen("rosrun tf tf_echo /base_link /{}".format(object_name), shell=True, stdout=subprocess.PIPE)
    # we listen for a max of 2 second
    current_time = time.time()
    # print out the object name in blue for debugging
    # blue text
    print("\033[34m{}\033[0m".format(object_name))

    translation = None
    rotation = None
    while time.time() - current_time < 2.0:
        line = tf_out.stdout.readline()
        # print out the line in yellow for debugging
        # yellow text
        print("\033[33m{}\033[0m".format(line))
        # we look for the line that contains the translation
        if b"Translation" in line:
            # the translation is in square brackets, separated by commas
            # we split the line by the square brackets and commas
            # the translation is the second element in the list
            translation = line.split(b"[")[1].split(b"]")[0].split(b",")
            # convert the translation to a list of floats
            translation = [float(x) for x in translation]
            # print out the translation in green for debugging
            # green text
            print("\033[32m{}\033[0m".format(translation))
        if b"Quaternion" in line:
            # the rotation is in square brackets, separated by commas
            # we split the line by the square brackets and commas
            # the rotation is the second element in the list
            rotation = line.split(b"[")[1].split(b"]")[0].split(b",")
            # convert the rotation to a list of floats
            rotation = [float(x) for x in rotation]
            # print out the rotation in green for debugging
            # green text
            print("\033[32m{}\033[0m".format(rotation))
        if translation is not None and rotation is not None:
            # we found the translation and rotation
            break
    # kill the tf_echo node
    tf_out.kill()
    return translation, rotation


def parse_object_position(position_text):
    # the position consists of 3 numbers, separated by a comma
    # print out the position for debugging
    # yellow text
    print("\033[33m")
    print("Position: {}".format(position_text))
    # reset color
    print("\033[0m")
    # sanitize the position text by removing the brackets
    position_text = position_text.replace("[", "").replace("]", "")
    position = [float(x) for x in position_text.split(",")]
    return position

def parse_pose(pose_text):
    # the pose consists of 7 numbers, the first 3 are the position, the last 4 are the orientation
    # the position and orientation are separated by a semicolon
    # the numbers are separated by a space
    # print out the pose for debugging
    # yellow text
    print("\033[33m")
    print("Pose: {}".format(pose_text))
    # reset color
    print("\033[0m")
    position_text, orientation_text = pose_text.split(";")
    position = [float(x) for x in position_text.split(" ")]
    orientation = [float(x) for x in orientation_text.split(" ")]
    return position, orientation



def find_closest_object(target_object_position, obj_positions):
    # find the object that is closest to the target object
    # in turn, this object is most likely the object that was targeted
    closest_distance = float("inf")
    closest_object = None
    for object_name, object_position in obj_positions.items():
        # print out the distance for debugging
        # yellow text
        print("\033[33m")
        print("Target object position: {}".format(target_object_position))
        print("Object position: {}".format(object_position))
        distance = np.linalg.norm(np.array(target_object_position) - np.array(object_position))
        print("Distance to {}: {}".format(object_name, distance))
        # reset color
        print("\033[0m")
        if distance < closest_distance:
            closest_distance = distance
            closest_object = object_name
    return closest_object

def run_one_trial(sim, scene_metadata, target_object_name, scene_name, trial_number, use_regrasp, logging_file):
    i = trial_number
    sim.stopSimulation()
    # also stop the moveit package, if it is running
    if os.system("rosnode list | grep move_group") == 0:
        os.system("rosnode kill /move_group")
    # as well as the main pipeline
    if os.system("rosnode list | grep main_pipeline") == 0:
        os.system("rosnode kill /main_pipeline")
    # make sure everything is stopped
    time.sleep(1)
    # experiment flags
    NO_GRASP_FLAG = False
    CONTROLLER_FAILURE_FLAG = False
    NO_SOLUTION_FLAG = False

    # restart simulation and moveit
    sim.startSimulation()
    # start moveit in a separate process,do not print its output, it should not accept signals
    planner = subprocess.Popen("roslaunch fetch_moveit_config move_group.launch", shell=True,cwd=WS_BASE,
        stdin=subprocess.PIPE, stdout=None, stderr=None, close_fds=True, preexec_fn=os.setsid)

    # wait for simulation to start
    print("Waiting for simulation to start...")
    time.sleep(1)
    obj_init_poses = {}
    for object_name in scene_metadata["objects"]:
        obj_init_poses[object_name] = get_object_pose(object_name)
    # print out the initial poses for debugging
    # yellow text
    print("\033[33m")
    print("Initial poses: {}".format(obj_init_poses))
    # reset color

    # start the main pipeline
    main_output = subprocess.Popen("rosrun manipulation_test main_pipeline _use_regrasp:="+str(use_regrasp).lower(), 
        shell=True, cwd=WS_BASE, stdout=subprocess.PIPE, stdin=subprocess.PIPE)
    obj_positions = {}

    # wait until the pipeline asks for user input, read the output of the pipeline
    while True:
        # check if the pipeline is still running
        if main_output.poll() is not None:
            # if the pipeline is not running, there is something wrong
            print("\033[31m")
            print("Pipeline is not running")
            print("\033[0m")
            return 1
        output = main_output.stdout.readline()
        # convert bytestring to string
        output = output.decode("utf-8")
        # # remove newline character
        # output = output.rstrip()
        print(output, end="")
        if output:
            if "obstacle position is: " in output:
                object_id = int(output.split("obstacle position is: ")[0])
                object_position = parse_object_position(output.split("obstacle position is: ")[1])
                # print out the position of the obstacle for debugging
                # yellow text
                print("\033[33m")
                print("Object position: {}".format(object_position))
                # reset color
                obj_positions[object_id] = object_position

            if "Please enter the grasped object id: " in output:
                print("Selecting object id for target object" + target_object_name)
                # find the id of the object that is closest to the target object
                target_object_id = find_closest_object(obj_init_poses[target_object_name][0], obj_positions)
                # print the id of the target object for debugging
                # yellow text
                print("\033[33m")
                print("Target object id: {}".format(target_object_id))
                # reset color
                print("\033[0m")
                # write bytestring to stdin,ending with a newline
                main_output.stdin.write(str(target_object_id).encode("utf-8") + b"\n")
                main_output.stdin.flush()
                break
    # print the output of the pipeline until it finishes
    # if the arm closes, read in the target object's pose
    logs = []
    target_object_pose_after_grasp = None
    while True:
        output = main_output.stdout.readline()
        # convert bytestring to string
        output = output.decode("utf-8")
        # log for checking if the there is a failure in the pipeline    
        print(output, end="")
        logs.append(output)
        # TODO: need to modify the main pipeline so that it waits for ~1 second before it starts moving the arm
        if "move arm to lift object" in output:
            # get the pose of the target object from tf
            target_object_pose_after_grasp = get_object_pose(target_object_name)
            # print out the pose of the target object for debugging
            # yellow text
            print("\033[33m")
            print("Target object pose after grasp: {}".format(target_object_pose_after_grasp))
            # reset color
        if output == '' and main_output.poll() is not None:
            break
        
    print("Run {} for scene {} finished".format(i, scene_name) )
    # check for success
    if target_object_name is not None:
        # get the pose of the target object from tf
        target_object_pose = get_object_pose(target_object_name)
        # print out the initial pose of the target object and the final pose in yellow for debugging
        # yellow
        print("\033[93m")
        print("Target object name: {}".format(target_object_name))
        print("Initial pose of target object: {}".format(obj_init_poses[target_object_name]))
        print("Final pose of target object: {}".format(target_object_pose))
        # reset color
        print("\033[0m")

        # get the pose of the finger from tf
        finger_position, _ = get_object_pose("l_gripper_finger_link")
        target_object_position = target_object_pose[0]
        # check the distance between the finger and the target object, they should be close
        distance = np.linalg.norm(np.array(target_object_position) - np.array(finger_position))
        # also check if the target object has been lifted
        z_diff_from_init = target_object_position[2] - obj_init_poses[target_object_name][0][2]
        # TODO: make this detection more robust
        success = distance < 0.15 and z_diff_from_init > 0.05
        # also store the difference in rotation 
        if target_object_pose_after_grasp is not None:
            # first unitize the rotation quaternions
            lifting_unit_quaternion = target_object_pose_after_grasp[1] / np.linalg.norm(target_object_pose_after_grasp[1])
            final_unit_quaternion = target_object_pose[1] / np.linalg.norm(target_object_pose[1])
            # then compute the difference in rotation
            sqrt_rot_diff = np.linalg.norm(lifting_unit_quaternion[1] - final_unit_quaternion[1])
        else:
            sqrt_rot_diff = None
        if success:
            # green text
            print("\033[92m")
            print("Run {} for scene {} succeeded".format(i, scene_name) )
            print("Distance between finger and target object: {}".format(distance))
            print("Z difference between target object and initial pose: {}".format(z_diff_from_init))
            print("Difference in rotation: {}".format(sqrt_rot_diff))
            # reset color
            print("\033[0m")
        else:
            # red text
            print("\033[91m")
            print("Run {} for scene {} failed".format(i, scene_name) )
            print("Distance between finger and target object: {}".format(distance))
            print("Z difference between target object and initial pose: {}".format(z_diff_from_init))
            print("Difference in rotation: {}".format(sqrt_rot_diff))
            # reset color
            print("\033[0m")
            # check in logs if there was a controller failure
            for log in logs:
                if "controller fail" in log:
                    CONTROLLER_FAILURE_FLAG = True
                if "No lifting grasp found by Contact Grasp Net, breaking" or "no way to grasp the object" in log:
                    NO_GRASP_FLAG = True
                if "no solution is found" in log:
                    NO_SOLUTION_FLAG = True
        # log the result
        logging_file.write("Run {}: {}\n".format(i, success))
        logging_file.write("Distance between finger and target object: {}\n".format(distance))
        logging_file.write("Z difference between target object and initial pose: {}\n".format(z_diff_from_init))
        logging_file.write("Difference in rotation: {}\n".format(sqrt_rot_diff))
        logging_file.write("CONTROLLER_FAILURE_FLAG: {}\n".format(CONTROLLER_FAILURE_FLAG))
        logging_file.write("NO_GRASP_FLAG: {}\n".format(NO_GRASP_FLAG))
        logging_file.write("NO_SOLUTION_FLAG: {}\n".format(NO_SOLUTION_FLAG))
        logging_file.flush()
    
    else:
        print("No target object found")
        logging_file.write("Run {}: {}\n".format(i, False))
        logging_file.write("No target object found\n")
        logging_file.flush()
        return 1
    return 0

def reset_arm(joint_names, init_position, robot_controller_client):
    # reset the arm with joint trajectory controller
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    # point.velocities.append(0.1)
    point.positions = init_position
    point.time_from_start = rospy.Duration(1)
    goal.trajectory.points.append(point)
    robot_controller_client.send_goal_and_wait(goal)
    # wait for 5 seconds
    time.sleep(5)

def run_for_scene(scene_name, constant_weights=True, variable_weights=True):
    # ascii box with scene name in the middle
    print("+" + "-" * len(scene_name) + "+")
    print("|" + "\033[96m{}\033[0m".format(scene_name) + "|")
    print("+" + "-" * len(scene_name) + "+")
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    sim.stopSimulation()
    time.sleep(1)
    scene_path = os.path.expanduser('~') + "/catkin_ws/src/jiaming_manipulation/fetch_coppeliasim/scene"
    sim.loadScene(os.path.join(scene_path, scene_name+".ttt"))
    scene_metadata = pickle.load(open(os.path.join(scene_path, scene_name+".pkl"), "rb"))
    print(scene_metadata)
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
    robot_controller_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    init_position = [-1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    print("Waiting for controller to start")
    robot_controller_client.wait_for_server()
    print("Controller started")
    # log the scene name in experiments.log
    logging_file = open(os.path.join(WS_BASE, "src/jiaming_manipulation/experiments.log"), "a")
    # log scene name and time date
    logging_file.write("Scene: {}\n".format(scene_name))
    logging_file.write("Scene path: {}\n".format(os.path.join(scene_path, scene_name+".ttt")))
    logging_file.write("Date: {}\n".format(datetime.datetime.now()))
    logging_file.flush()
    start_time = time.time()
    # non-weight varying grasp task
    if constant_weights:
        # log the experiment type in experiments.log
        logging_file.write("Experiment type: Constant weight\n")
        for target_object_name in scene_metadata["target_object_names"]:
            # Green text
            print("\033[32m")
            print("===========================================")
            print("Target object: {}".format(target_object_name))
            # reset color
            print("\033[0m")
            # log the target object name in experiments.log
            logging_file.write("Target object: {}\n".format(target_object_name))
            for i in range(2):
                use_regrasp = False
                if i == 0:
                    # Green text
                    print("\033[32m")
                    print("No regrasp")
                    # reset color
                    print("\033[0m")
                    # log the experiment type in experiments.log
                    logging_file.write("Experiment type: No regrasp\n")
                    use_regrasp = False
                else:
                    # Green text
                    print("\033[32m")
                    print("With regrasp")
                    # reset color
                    print("\033[0m")
                    # log the experiment type in experiments.log
                    logging_file.write("Experiment type: With rerasp\n")
                    use_regrasp = True

                for i in range(5):
                    # Green text
                    print("\033[32m")
                    print("Run {}".format(i))
                    # reset color
                    print("\033[0m")
                    retval = run_one_trial(sim, scene_metadata, target_object_name, scene_name, i, use_regrasp, logging_file)
                    if retval == 1:
                        return 1
                    reset_arm(joint_names, init_position, robot_controller_client)
    if variable_weights:
        #reset scene
        sim.stopSimulation()
        time.sleep(1)
        sim.loadScene(os.path.join(scene_path, scene_name+".ttt"))
        # weight varying grasp task
        # log the experiment type in experiments.log
        logging_file.write("Experiment type: Weight varying\n")
        for target_object_name in scene_metadata["target_object_weights"]:
            # log the target object name in experiments.log
            logging_file.write("Target object: {}\n".format(target_object_name))
            handle_name = scene_metadata["target_object_weights"][target_object_name][0]
            obj_handle = sim.getObjectHandle(handle_name)
            weights = scene_metadata["target_object_weights"][target_object_name][1]
            # if weights is empty, use default weight scaling scheme
            if len(weights) == 0:
                original_weight = sim.getShapeMassAndInertia(obj_handle)[0]
                weights = [original_weight, original_weight *1.5, original_weight * 2.0]
            # set the weight of the target object
            for weight in weights:
                # debugging print 
                # green text
                print("\033[32m")
                print("===========================================")
                print("Target object: {}".format(target_object_name) + " Weight: {}".format(weight))
                # reset color
                print("\033[0m")
                # run 1 trial(s) for each weight with no regrasp and regrasp
                sim.setShapeMass(obj_handle, weight)
                # TODO: make sure this actually works, as inertia is not set
                logging_file.write("No regrasp")
                retval = run_one_trial(sim, scene_metadata, target_object_name, scene_name, 0, False, logging_file)
                if retval == 1:
                    return 1
                reset_arm(joint_names, init_position, robot_controller_client)
                logging_file.write("With regrasp")
                retval = run_one_trial(sim, scene_metadata, target_object_name, scene_name, 0, True, logging_file)
                if retval == 1:
                    return 1
                reset_arm(joint_names, init_position, robot_controller_client)
    
    # all experiments are done, print the end of the experiment
    # green text
    print("\033[32m")
    print("===========================================")
    print("All experiments are done for scene {}".format(scene_name))
    print("took {} minutes".format((time.time() - start_time)/60.0))
    # reset color
    print("\033[0m")
    # log the end of the experiment in experiments.log
    logging_file.write("All experiments are done for scene {}\n".format(scene_name))
    # also, log the time date
    logging_file.write("Date: {}\n".format(datetime.datetime.now()))
    logging_file.write("===========================================")
    logging_file.flush()

if __name__ == "__main__":
    rospy.init_node("experiment_monitor")
    run_for_scene("tableroom_1", constant_weights=True, variable_weights=False)

        

