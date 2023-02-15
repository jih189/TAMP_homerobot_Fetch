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
# also stop the moveit package, if it is running
if os.system("rosnode list | grep move_group") == 0:
    os.system("rosnode kill /move_group")
# make sure simulation is stopped
time.sleep(2)

# load scene TODO: make this a parameter or something
scene_path = "/home/lambda/catkin_ws/src/jiaming_manipulation/fetch_coppeliasim/scene"
sim.loadScene(os.path.join(scene_path, "tableroom.ttt"))

# restart simulation and moveit
sim.startSimulation()
# start moveit in a separate process
subprocess.Popen("roslaunch fetch_moveit_config move_group.launch", shell=True,
    stdin=None, stdout=None, stderr=None, close_fds=True, preexec_fn=os.setsid)

# wait for simulation to start
print("Waiting for simulation to start...")
time.sleep(2)

# start the main pipeline
os.system("rosrun manipulation_test main_pipeline")
# TODO: automatic object selection

