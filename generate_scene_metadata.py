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
    "target_object_names": ["candy"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem2)

scene_metadatem3 = {
    "scene_name": "tableroom_3",
    "objects": ['laptop', 'hairdryer', 'mic', 'earphone'],
    "target_object_names": ["mic"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem3)

scene_metadatem4 = {
    "scene_name": "tableroom_4",
    "objects": ['oil', 'holetool', 'measuretool', 'weight'],
    "target_object_names": ["measuretool"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem4)

scene_metadatem5 = {
    "scene_name": "tableroom_5",
    "objects": ['light', 'wrench', 'hammer', 'drill', 'laptop'],
    "target_object_names": ["wrench"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem5)

scene_metadatem6 = {
    "scene_name": "tableroom_6",
    "objects": ['stable', 'lego', 'tissue', "clock", "pen"],
    "target_object_names": ["tissue"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem6)

scene_metadatem7 = {
    "scene_name": "tableroom_7",
    "objects": ['oil', 'holetool', 'measuretool', "weight"],
    "target_object_names": ["weight"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem7)

scene_metadatem8 = {
    "scene_name": "tableroom_8",
    "objects": ['cereal', 'remote', 'dispenser'],
    "target_object_names": ["cereal"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem8)

scene_metadatem9 = {
    "scene_name": "tableroom_9",
    "objects": ['book', 'candy', 'bottle', 'can'],
    "target_object_names": ["can"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem9)

scene_metadatem10 = {
    "scene_name": "tableroom_10",
    "objects": ['stable', 'tissue', "clock", "pen"],
    "target_object_names": ["clock"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem10)

scene_metadatem11 = {
    "scene_name": "tableroom_11",
    "objects": ['laptop', 'hairdryer', 'mic', 'earphone', 'holetool'],
    "target_object_names": ["mic"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem11)

scene_metadatem12 = {
    "scene_name": "tableroom_12",
    "objects": ['light', 'wrench', 'hammer', 'holetool'],
    "target_object_names": ["holetool"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem12)

scene_metadatem13 = {
    "scene_name": "tableroom_13",
    "objects": ['oil', 'holetool', 'measuretool', 'weight'],
    "target_object_names": ["weight"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem13)

scene_metadatem14 = {
    "scene_name": "tableroom_14",
    "objects": ['stable', 'lego', 'clock'],
    "target_object_names": ["lego"],
    "target_object_weights": {},
}
scene_metadata.append(scene_metadatem14)

SCENE_DIR = os.path.expanduser('~') + "/catkin_ws/src/jiaming_manipulation/fetch_coppeliasim/scene"

for scene in scene_metadata:
    scene_name = scene["scene_name"]
    scene_path = SCENE_DIR
    scene_file = os.path.join(scene_path, scene_name + ".pkl")
    with open(scene_file, 'w+b') as f:
        pickle.dump(scene, f)

