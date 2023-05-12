#!/usr/bin python3.8
''' A script to plan using 7D robot
'''
import rospy
from moveit_msgs.srv import GetNextStep, GetNextStepResponse
from sensor_msgs.msg import PointCloud2
from utils import pointcloud2_to_xyz_array

from torch.nn import functional as F

import time
import skimage.io
from os import path as osp

import numpy as np
import torch
import json
import argparse
import pickle
import open3d as o3d
import torch_geometric.data as tg_data

import matplotlib.pyplot as plt

try:
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import util as ou
except ImportError:
    raise "Run code from a container with OMPL installed"

import fetch_utils as fu

device = torch.device('cuda') if torch.cuda.is_available() else 'cpu'



class NextStepPredServer:
    def __init__(self):
        # ======================== Model loading :START ========================================
        # Load the point cloud encoder model.
        from CAE_JM import Encoder
        self.encoder_model = Encoder()
        self.encoder_model.load_state_dict(torch.load('models/cae_encoder.pkl'))
        if torch.cuda.is_available():
            self.encoder_model.cuda()
        

        
        # load the planner model
        from model import MLP
        self.planner_model = MLP(input_size=42, output_size=7)
        self.planner_model.load_state_dict(torch.load('models/mlp_100_4000_PReLU_ae_dd_final.pkl'))
        if torch.cuda.is_available():
            self.planner_model.cuda()

        
        
        self.server = rospy.Service('next_step_predict', GetNextStep, self.handle_request)
        self.pc_subscriber = rospy.Subscriber("obstacle_point_cloud", PointCloud2, self.point_cloud_callback, queue_size=1)

        self.obstacle_pc = None

        # also load the normalization parameters
        with open('env_1000_range.pkl', 'rb') as f:
            self.norm_params = pickle.load(f)
        

    def handle_request(self, req):
        print("receive req")
        # print(req.start_configuration)
        # print(req.goal_configuration)

        # wait for the obstacle point cloud is ready
        wait_start = time.time()
        while(self.obstacle_pc is None):
            time.sleep(0.5)
            if time.time() - wait_start > 5.0:
                print("wait for obstacle point cloud timeout")
                return GetNextStepResponse()

        # set the pointcloud into tensor
        pc = self.obstacle_pc
        pc = pc.T # (3,N) -> (N,3)
        # resample the point cloud to 2000 points
        if pc.shape[0] > 2000:
            pc = pc[np.random.choice(pc.shape[0], 2000, replace=False),:]
        elif pc.shape[0] < 2000:
            pc = np.concatenate([pc, np.zeros((2000-pc.shape[0],3))], axis=0)
        pc = torch.from_numpy(pc).float().unsqueeze(0).to(device)
        # print(pc.shape)

        # encode the point cloud
        with torch.no_grad():
            pce = self.encoder_model(pc)

        
        # normalize the start and goal configuration and set them into tensors
        tmp = (np.array([req.start_configuration, req.goal_configuration])-self.norm_params["mid"])/(self.norm_params["range"])
        tmp = torch.from_numpy(tmp).float().to(device)

        # concatenate the start and goal configuration with the point cloud encoding
        input = torch.cat([pce, tmp], dim=1)

        # predict the next step
        with torch.no_grad():
            pred = self.planner_model(input)

        # denormalize the prediction
        result = pred.cpu().numpy()*self.norm_params["range"]+self.norm_params["mid"]        

        return result

    def point_cloud_callback(self, point_cloud):
        self.obstacle_pc = pointcloud2_to_xyz_array(point_cloud)


if __name__ == "__main__":



    # ======================== Model loading :END ========================================
    # ======================== Data loading : START ======================================
    rospy.init_node('next_step_prediction_server', anonymous=True)
    
    try:
        server = NextStepPredServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass