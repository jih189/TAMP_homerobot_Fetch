import pickle
import numpy as np
import os

# this file is currently a stub
# TODO: use this file to automate scene metadata generation

scene_metadata = {
    "objects": ['bottle', 'can', 'book', 'hammer'],
    "target_object_ids": [1,3],
}

SCENE_DIR = "/home/lambda/catkin_ws/src/jiaming_manipulation/fetch_coppeliasim/scene"

with open(os.path.join(SCENE_DIR, "tableroom.pkl"), "wb") as f:
    pickle.dump(scene_metadata, f)

