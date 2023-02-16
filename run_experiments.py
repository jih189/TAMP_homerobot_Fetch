import os
import sys
import subprocess
# add the coppeliasim remote api path
sys.path.append("/home/lambda/CoppeliaSim/programming/zmqRemoteApi/clients/python/")
print(sys.path)

# coppeliasim imports
import time
from zmqRemoteApi import RemoteAPIClient

WS_BASE = "/home/lambda/catkin_ws"
# change the working directory to the base of the workspace
# os.chdir(WS_BASE)

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.stopSimulation()
time.sleep(1)
# load scene TODO: make this a parameter or something
scene_path = "/home/lambda/catkin_ws/src/jiaming_manipulation/fetch_coppeliasim/scene"
sim.loadScene(os.path.join(scene_path, "tableroom.ttt"))


for i in range(5):
    sim.stopSimulation()
    # make sure simulation is stopped
    # also stop the moveit package, if it is running
    if os.system("rosnode list | grep move_group") == 0:
        os.system("rosnode kill /move_group")
    # make sure simulation is stopped
    time.sleep(1)



    # restart simulation and moveit
    sim.startSimulation()
    # start moveit in a separate process,do not print its output
    subprocess.Popen("roslaunch fetch_moveit_config move_group.launch", shell=True,
        stdin=None, stdout=None, stderr=None, close_fds=True, preexec_fn=os.setsid)

    # wait for simulation to start
    print("Waiting for simulation to start...")
    time.sleep(1)

    # start the main pipeline
    main_output = subprocess.Popen("rosrun manipulation_test main_pipeline", shell=True, stdout=subprocess.PIPE, stdin=subprocess.PIPE)
    # TODO: automatic object selection
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
    while True:
        output = main_output.stdout.readline()
        # convert bytestring to string
        output = output.decode("utf-8")
        print(output)
        if output == '' and main_output.poll() is not None:
            break
        
    print("Run {} for scene {} finished".format(i, "tableroom.ttt") )
    # TODO: devise a way to getthe result of the run and log it



    

