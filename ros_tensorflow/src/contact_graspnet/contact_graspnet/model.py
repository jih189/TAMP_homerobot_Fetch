#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np
import time
import glob
import cv2

import tensorflow.compat.v1 as tf
tf.disable_eager_execution()
physical_devices = tf.config.experimental.list_physical_devices('GPU')
for gpu_id in range(len(physical_devices))
    tf.config.experimental.set_memory_growth(physical_devices[gpu_id], True)
# tf.config.experimental.set_memory_growth(physical_devices[1], True)

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(BASE_DIR))
from . import config_utils
from .data import regularize_pc_point_count, depth2pc, load_available_input_data

from contact_grasp_estimator import GraspEstimator
# from visualization_utils import visualize_grasps, show_image

class ModelWrapper():
    def __init__(self):

        checkpoint_dir = os.path.expanduser('~') + '/catkin_ws/src/jiaming_manipulation/ros_tensorflow/src/contact_graspnet/checkpoints/scene_test_2048_bs3_hor_sigma_001'
        global_config = config_utils.load_config(checkpoint_dir, batch_size=1, arg_configs=[])
        print(str(global_config))
        print('pid: %s'%(str(os.getpid())))
        # use GPU 1, if possible
        device_name = '/gpu:0'
        # check if there are more than 1 gpu
        for device in tf.config.list_physical_devices():
            if "GPU:1" in device.name:
                device_name = '/gpu:1'
        with tf.device(device_name):
            self.grasp_estimator = GraspEstimator(global_config)
            self.grasp_estimator.build_network()

            # Add ops to save and restore all the variables.
            saver = tf.train.Saver(save_relative_paths=True)
            
            # Create a session
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            config.allow_soft_placement = True
            self.sess = tf.Session(config=config)

            # Load weights
            self.grasp_estimator.load_weights(self.sess, saver, checkpoint_dir, mode='test')

    def predict(self, pc_full, pc_segment):
        pc_segments = {}
        pc_segments[1.0] = pc_segment
        pred_grasp_cam, scores, _, _ = self.grasp_estimator.predict_scene_grasps(self.sess, pc_full, pc_segments=pc_segments, local_regions=True, filter_grasps=True, forward_passes=1)
        return pred_grasp_cam[1.0], scores[1.0]
