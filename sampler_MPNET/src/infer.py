import argparse
import torch
import torch.nn as nn
import numpy as np
import os
import pickle
from data_loader import load_dataset_JM 
from model import MLP 
from torch.autograd import Variable 
import math
import time

# Load trained model for path generation
mlp = MLP(42, 7) 
mlp.load_state_dict(torch.load('models/mlp_100_4000_PReLU_ae_dd_final.pkl'))

if torch.cuda.is_available():
    mlp.cuda()

#load test dataset
dataset,targets= load_dataset_JM() 
criterion = nn.MSELoss()
# run 10 different tests
for i in range(10):
    input = dataset[i:i+1]
    input = torch.from_numpy(input)
    input = input.cuda()
    input = Variable(input)
    target = targets[i:i+1]
    target = torch.from_numpy(target)
    target = target.cuda()
    target = Variable(target)
    output = mlp(input)
    loss = criterion(output, target).cpu().data.numpy()
    print("target: ", target)
    print("output: ", output)
    print((output - target)**2)
    print("loss: ", np.abs(loss))

# import open3d as o3d

# for i in range(10):
#     input = dataset[i]
#     # randomly change the last 14 dimensions of the input
#     input[28:] = np.random.rand(14)
#     input = torch.from_numpy(input)
#     input = input.cuda()
#     input = Variable(input)
#     output = mlp(input).cpu().data.numpy()
#     print("output: ", output)
    
