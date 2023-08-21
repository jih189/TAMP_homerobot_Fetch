import torch
from torch.utils.data import Dataset
import os
import glob
import numpy as np
from sklearn import mixture
from point_cloud_utils import read_ply
from torch.utils.data import DataLoader

class GMMDataset(Dataset):

    def __init__(self, dir_path, split_list, gmm_path = None, ):
    
        self.env_dir = dir_path
        self.split_list = split_list
        arm_config_gmms, self.num_distributions = self.get_distributions(np.load(os.path.join(dir_path, "valid_robot_states.npy")), gmm_path)
        self.arm_config_gmm_indices = []
        zero_loss_masks = []
        for i in range(self.num_distributions):
            dist_configs = np.argwhere(arm_config_gmms==i)[:, 0].tolist()
            self.arm_config_gmm_indices.append(dist_configs)
            zero_loss_masks.append(len(dist_configs))
        print("Arm config indices : ", self.arm_config_gmm_indices)
        self.zero_loss_masks = np.array(zero_loss_masks)

    def get_distributions(self, arm_configs, gmm_dir_name = "../gmm/"):

        means = np.load(gmm_dir_name + 'means.npy')
        covariances = np.load(gmm_dir_name + 'covariances.npy')
        # Create an sklearn Gaussian Mixture Model 
        sklearn_gmm_ = mixture.GaussianMixture(n_components = len(means), covariance_type='full')
        sklearn_gmm_.precisions_cholesky_ = np.linalg.cholesky(np.linalg.inv(covariances))
        sklearn_gmm_.weights_ = np.load(gmm_dir_name + 'weights.npy')
        sklearn_gmm_.means_ = means
        sklearn_gmm_.covariances_ = covariances
        print("Loaded %d distributions " % len(means))
        print("arm configs shape : ", arm_configs.shape)
        predicted_gmms = sklearn_gmm_.predict(arm_configs[:, 6:13])
        return predicted_gmms, len(means)

    def __getitem__(self, index):

        env_dir = os.path.join(self.env_dir, 'env_%06d'%self.split_list[index])
        pc = read_ply(os.path.join(env_dir, 'map_%d.ply'%self.split_list[index]))[0] # return only xyz and not colors
        valid_configs_labels = np.load(os.path.join(env_dir, 'valid_tag.npy'))

        labels = []
        for dist_no, arm_config_gmm_index in enumerate(self.arm_config_gmm_indices):
            gmm_valid_val = valid_configs_labels[arm_config_gmm_index]
            if self.zero_loss_masks[dist_no]:
                labels.append(gmm_valid_val.sum() / len(gmm_valid_val))
            else:
                labels.append(0.0)
        return pc, self.zero_loss_masks, np.array(labels)

    def __len__(self):
        return len(self.split_list)


def test_read_ply_file():
    filename = "gmm_data/env_000000/map_0.ply"
    X, c = read_ply(filename)
    print(X)
    print(c)

def test_create_gmm_dataset():
    dataset = GMMDataset(dir_path="gmm_data", split_list=list(range(5000)), gmm_path="computed_gmms_dir/dpgmm/")
    X, loss_mask, labels = dataset.__getitem__(100)
    print(X.shape, labels.shape, loss_mask.shape)


def create_dataloader():

    generated_data_dir = "gmm_data"
    train_frac, test_frac, val_frac = (0.8, 0.1, 0.01)

    no_of_envs = len(os.listdir(generated_data_dir)) - 2
    random_list = np.random.choice(no_of_envs, no_of_envs)

    train_list = random_list[:int(no_of_envs * train_frac)]
    test_list = random_list[len(train_list) : len(train_list) + int(no_of_envs * test_frac)]
    val_list = random_list[len(test_list) + len(train_list) : len(test_list) + len(train_list) + int(no_of_envs * val_frac)]
    
    print(len(train_list), len(test_list), len(val_list))

    train_dataset = GMMDataset(dir_path=generated_data_dir, split_list=train_list, gmm_path="computed_gmms_dir/dpgmm/")
    train_loader = DataLoader(dataset=train_dataset,
                            batch_size=16,
                            drop_last=True,
                            shuffle=True, # want to shuffle the dataset
                            num_workers=2) # number processes/CPUs to use    


    val_dataset = GMMDataset(dir_path=generated_data_dir, split_list=val_list, gmm_path="computed_gmms_dir/dpgmm/")
    val_loader = DataLoader(dataset=val_dataset,
                            batch_size=16,
                            drop_last=True,
                            shuffle=True, # want to shuffle the dataset
                            num_workers=2) # number processes/CPUs to use    

    return train_loader, val_loader

def test_dataloader():

    train_loader = create_dataloader()
    print(dir(train_loader.dataset))

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(0)

    num_epochs = 2
    for epoch in range(num_epochs):

        for batch_idx, (pc, mask, label) in enumerate(train_loader):
            
            print('Epoch:', epoch+1, end='')
            print(' | Batch index:', batch_idx, end='')
            print(' | Batch size:', label.size()[0])
            
            pc = pc.to(device)
            label = label.to(device)

if __name__ == "__main__":
    # test_read_ply_file()
    test_dataloader()