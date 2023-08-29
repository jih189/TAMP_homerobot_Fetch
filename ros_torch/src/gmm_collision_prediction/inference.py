from ros_torch.src.gmm_collision_prediction.gmm_collision_model import create_model
import torch
import numpy as np
import trimesh
# from point_cloud_utils import display_point_cloud
from trimesh_utils import sample_points_on_mesh, filter_points_inside_mesh
from point_cloud_utils import normalize_pc

torch.set_printoptions(precision = 3, sci_mode = False)
np.set_printoptions(precision = 3, suppress = True)

def read_mesh(filename):
    mesh = trimesh.load_mesh(filename)
    pc = sample_points_on_mesh(mesh, 5000)
    pc = normalize_pc(pc)
    return pc


def transform_cloud(pc):
    T = np.array([[0.0, 1.0, 0.0, 0.28], [-1.0, 0.0, 0.0, 0.92], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    print(pc.shape)
    return (T[:3, :3] @ pc.T + np.expand_dims(T[:3, 3], axis = 1)).T



if __name__ == '__main__':

    # import  torch
    gmm_model = create_model()

    checkpoint = torch.load("Pointnet2_PyTorch/outputs/cls-msg/epoch=107-val_loss=0.54-val_acc=0.535.ckpt")
    # checkpoint = torch.load("Pointnet2_PyTorch/outputs/cls-msg/epoch=101-val_loss=0.55-val_acc=0.531.ckpt")
    
    gmm_model.load_state_dict(checkpoint['state_dict'])
    gmm_model.eval()
    gmm_model.to(device = "cuda:0")

    pc_np = read_mesh("/src/table.stl")
    pc_np = transform_cloud(pc_np)
    pc_torch = torch.tensor(pc_np).to(device = "cuda:0")
    pc_torch = torch.reshape(pc_torch, (1, -1, 3)).float()
    output = torch.sigmoid(gmm_model(pc_torch)).data.cpu().reshape(-1).numpy()
    labels = np.load("labels.npy")
    out = np.vstack([output, labels])
    loss = ((output - labels) ** 2)
    eval = loss < 0.01

    print(out.T)
    print(eval.sum() / len(eval), np.sqrt(loss.mean()))
