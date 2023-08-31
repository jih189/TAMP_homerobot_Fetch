#!/usr/bin/env python3

#print the python version
import sys
print(sys.version)
import os
import torch
import numpy as np
from gmm_collision_prediction.gmm_collision_model import create_model
from gmm_collision_prediction.point_cloud_utils import normalize_pc

# Global variables
BATCH_SIZE = 1
NUM_POINT = 7000
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
print("BASE_DIR: ", BASE_DIR)


class ModelWrapper():
    def __init__(self):
        # get the model

        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.gmm_model = create_model()
        checkpoint = torch.load(os.path.join(BASE_DIR, "checkpoints/epoch=107-val_loss=0.54-val_acc=0.535.ckpt"))
        self.gmm_model.load_state_dict(checkpoint['state_dict'])
        self.gmm_model.eval()
        self.gmm_model.to(device = self.device)


    def predict(self, x):
        assert x.shape[1] == 3, "Incorrect point cloud channels"
        assert len(x.shape) == 2, "Incorrect point cloud shape"
        x = normalize_pc(x)
        pc_torch = torch.tensor(x).to(device = self.device)
        pc_torch = torch.reshape(pc_torch, (1, -1, 3)).float()
        output = torch.sigmoid(self.gmm_model(pc_torch)).data.cpu().reshape(-1).numpy()
        return output

