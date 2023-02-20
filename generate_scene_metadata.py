import pickle
import numpy as np
import os

# this file is currently a stub
# TODO: fill in the rest of the 19 scenes
'''
scene metadata is a list of dictionary, with each dictionary containing the following keys:
    scene_name <string>: name of the scene, must be the same as the .ttt file
    objects [<string>]: list of objects in the scene, must be the same as the name advertised in the /tf topic
    target_object_names [<string>]: list of objects that are the target of the manipulation task, must be a subset of objects
    target_object_weights {<string>:(<string>, [<int>])}: see below for explanation

target_object_weights is a dictionary, with each key being a target object name, and each value being a tuple of two elements:
    the first element is the path of the coppeliasim handle to the object
    the second element is a list of integers, where each integer is the list of weights in Kg for the object to be tested.
    PS: if the list of weights is empty, then the object will be ran through the test 3 times, the first time with its default weight, the second time with 1.5x its default weight, and the third time with 2.0x its default weight.
'''
scene_metadata = []

scene_metadatem0 = {
    "scene_name": "tableroom",
    "objects": ['bottle', 'can', 'book', 'hammer'],
    "target_object_names": ["hammer"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem0)

# TODO: fill in the rest of the 19 scenes here
scene_metadatem1 = {
    "scene_name": "tableroom_1",
    "objects": ['cereal', 'remote', 'dispenser', 'teapot', 'pan'],
    "target_object_names": ["pan"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem1)

scene_metadatem2 = {
    "scene_name": "tableroom_2",
    "objects": ['candy', 'stable', 'lego', 'tissue', 'clock'],
    "target_object_names": ["clock"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem2)

scene_metadatem3 = {
    "scene_name": "tableroom_3",
    "objects": ['candy', 'stable', 'lego', 'tissue', 'clock'],
    "target_object_names": ["clock"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem3)

scene_metadatem4 = {
    "scene_name": "tableroom_4",
    "objects": ['oil', 'holetool', 'measuretool', 'weight', 'pen'],
    "target_object_names": ["pen"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem4)


SCENE_DIR = "/home/lambda/catkin_ws/src/jiaming_manipulation/fetch_coppeliasim/scene"

for scene in scene_metadata:
    scene_name = scene["scene_name"]
    scene_path = SCENE_DIR
    scene_file = os.path.join(scene_path, scene_name + ".pkl")
    with open(scene_file, 'w+b') as f:
        pickle.dump(scene, f)

