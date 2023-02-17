
'''
This script acts as an experiment monitor, it runs the main pipeline and monitors the output of the pipeline.
'''

import os
import sys
import subprocess
import pickle
import rospy
import numpy as np
print(sys.version)
import time

def get_object_pose(object_name):
    # because tf doesn't play nice with python3, we have to directly call the tf_echo command
    # the tf_echo command returns a string with the pose of the object, we only want the translation
    tf_out = subprocess.Popen("rosrun tf tf_echo /base_link /{}".format(object_name), shell=True, stdout=subprocess.PIPE)
    # we listen for a max of 1 second
    current_time = time.time()
    # print out the object name in blue for debugging
    # blue text
    print("\033[34m{}\033[0m".format(object_name))

    while time.time() - current_time < 1.0:
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
            return translation
            
    return None


# add the coppeliasim remote api path
sys.path.append("/home/lambda/CoppeliaSim/programming/zmqRemoteApi/clients/python/")
print(sys.path)

# coppeliasim imports
from zmqRemoteApi import RemoteAPIClient

WS_BASE = "/home/lambda/catkin_ws"
# change the working directory to the base of the workspace
# os.chdir(WS_BASE)

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



def find_closest_object(target_object_position, obj_init_poses):
    # find the object that is closest to the target object
    # in turn, this object is most likely the object that was targeted
    closest_distance = float("inf")
    closest_object = None
    for object_name, object_position in obj_init_poses.items():
        distance = np.linalg.norm(np.array(target_object_position) - np.array(object_position))
        # print out the distance for debugging
        # yellow text
        print("\033[33m")
        print("Target object position: {}".format(target_object_position))
        print("Object position: {}".format(object_position))
        print("Distance to {}: {}".format(object_name, distance))
        # reset color
        print("\033[0m")
        if distance < closest_distance:
            closest_distance = distance
            closest_object = object_name
    return closest_object


rospy.init_node("experiment_monitor")
client = RemoteAPIClient()
sim = client.getObject('sim')
sim.stopSimulation()
time.sleep(1)
# load scene TODO: make this a parameter or something
scene_path = "/home/lambda/catkin_ws/src/jiaming_manipulation/fetch_coppeliasim/scene"
sim.loadScene(os.path.join(scene_path, "tableroom.ttt"))
scene_metadata = pickle.load(open(os.path.join(scene_path, "tableroom.pkl"), "rb"))

for i in range(5):
    sim.stopSimulation()
    # also stop the moveit package, if it is running
    if os.system("rosnode list | grep move_group") == 0:
        os.system("rosnode kill /move_group")
    # as well as the main pipeline
    if os.system("rosnode list | grep main_pipeline") == 0:
        os.system("rosnode kill /main_pipeline")
    # make sure everything is stopped
    time.sleep(1)

    # restart simulation and moveit
    sim.startSimulation()
    # start moveit in a separate process,do not print its output, it should not accept signals
    subprocess.Popen("roslaunch fetch_moveit_config move_group.launch", shell=True,cwd=WS_BASE,
        stdin=subprocess.PIPE, stdout=None, stderr=None, close_fds=True, preexec_fn=os.setsid)

    # wait for simulation to start
    print("Waiting for simulation to start...")
    time.sleep(1)
    # TODO: make objects in the scene non-dynamic before the arm resets (workaround: reset arm after each run instead of before, would require changes to the pipeline)
    # get initial pose of the objects from tf
    obj_init_poses = {}
    for object_name in scene_metadata["objects"]:
        obj_init_poses[object_name] = get_object_pose(object_name)

    # start the main pipeline
    main_output = subprocess.Popen("rosrun manipulation_test main_pipeline", shell=True, cwd=WS_BASE, stdout=subprocess.PIPE, stdin=subprocess.PIPE)
    # TODO: re-enable object dynamics after the arm resets
    # wait until the pipeline asks for user input, read the output of the pipeline every 0.5 seconds
    while True:
        # check if the pipeline is still running
        if main_output.poll() is not None:
            break
        output = main_output.stdout.readline()
        # convert bytestring to string
        output = output.decode("utf-8")
        print(output)
        if output:
            if "Please enter the grasped object id: " in output:
                print("Selecting object")
                # write bytestring to stdin
                main_output.stdin.write(b"1\n")
                main_output.stdin.flush()
                break
    # print the output of the pipeline until it finishes
    target_object_name = None
    identified_target_object = False
    while True:
        output = main_output.stdout.readline()
        # convert bytestring to string
        output = output.decode("utf-8")
        # check for the line that indicates the pose of the target object
        if not identified_target_object and "target object transform: " in output:
            # get the pose of the target object
            target_object_pose_text = output.split("target object transform: ")[1]
            target_object_position, target_object_orientation = parse_pose(target_object_pose_text)
            # print out the pose of the target object in yellow for debugging
            # yellow
            print("\033[93m")
            print("Target object position: {}".format(target_object_position))
            print("Target object orientation: {}".format(target_object_orientation))
            # reset color
            # pin down which object is the target object
            target_object_name = find_closest_object(target_object_position, obj_init_poses)
            print("Target object name: {}".format(target_object_name))

            print("\033[0m")
            identified_target_object = True
            
        print(output)
        if output == '' and main_output.poll() is not None:
            break
        
    print("Run {} for scene {} finished".format(i, "tableroom.ttt") )
    # check for success
    if target_object_name is not None:
        # get the pose of the target object from tf
        target_object_position = get_object_pose(target_object_name)
        # print out the initial pose of the target object and the final pose in yellow for debugging
        # yellow
        print("\033[93m")
        print("Target object name: {}".format(target_object_name))
        print("Initial pose of target object: {}".format(obj_init_poses[target_object_name]))
        print("Final pose of target object: {}".format(target_object_position))
        # reset color
        print("\033[0m")

        # get the pose of the finger from tf
        finger_position = get_object_pose("l_gripper_finger_link")
        # check the distance between the finger and the target object, they should be close
        distance = np.linalg.norm(np.array(target_object_position) - np.array(finger_position))
        # also check if the target object has been lifted
        z_diff_from_init = target_object_position[2] - obj_init_poses[target_object_name][2]
        # TODO: make this detection more robust
        success = distance < 0.15 and z_diff_from_init > 0.05
        if success:
            # green text
            print("\033[92m")
            print("Run {} for scene {} succeeded".format(i, "tableroom.ttt") )
            print("Distance between finger and target object: {}".format(distance))
            print("Z difference between target object and initial pose: {}".format(z_diff_from_init))
            # reset color
            print("\033[0m")
        else:
            # red text
            print("\033[91m")
            print("Run {} for scene {} failed".format(i, "tableroom.ttt") )
            print("Distance between finger and target object: {}".format(distance))
            print("Z difference between target object and initial pose: {}".format(z_diff_from_init))
            # reset color
            print("\033[0m")



    

