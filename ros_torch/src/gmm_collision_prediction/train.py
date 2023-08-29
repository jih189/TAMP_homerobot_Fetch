import torch.optim as optim
import torch.nn as nn
from gmm_collision_predictor import PointCloudCollisionPredictor
from data_utils import create_dataloader
import torch
from torch.utils.tensorboard import SummaryWriter
torch.set_default_tensor_type(torch.DoubleTensor)

from tqdm import tqdm

from point_cloud_utils import display_point_cloud


# training function
def train(model, train_dataloader, val_dataloader, optimizer, criterion, device, writer):
    print('Training')
    model.train()
    
    
    for epoch in range(1000):
        
        train_running_loss = 0.0
        counter = 0
        running_loss = 0
        for i, (pc, mask, label) in enumerate(tqdm(train_dataloader)):
            counter += 1
            pc = pc.to(device)
            label = label.to(device)
            mask = mask.to(device)
            optimizer.zero_grad()
            pred, trans_feat = model(pc)
            loss_gmm = criterion(pred, label) # Compute ()
            loss_gmm = torch.mean(mask * loss_gmm)
            running_loss += loss_gmm.item()
            train_running_loss += loss_gmm.item()
            writer.add_scalar("BatchLoss", loss_gmm, i)
            writer.add_scalar("TotalTrainRunningLoss", train_running_loss, counter * (epoch + 1))

            if ((counter % 50) == 0):
                print(running_loss)
                writer.add_scalar("RunningLoss", running_loss, (epoch + 1) * counter)
                running_loss = 0

            # backpropagation
            loss_gmm.backward()
            # update optimizer parameters
            optimizer.step()
        
        train_loss = train_running_loss / counter
        writer.add_scalar("TrainEpochLoss", train_loss, epoch)

        # Disable gradient computation and reduce memory consumption.
        running_vloss = 0.0
        # Set the model to evaluation mode, disabling dropout and using population
        # statistics for batch normalization.
        model.eval()
        correc_predictions = 0.0
        total_predictions = 0.0
        with torch.no_grad():
            for i, vdata in enumerate(tqdm(val_dataloader)):
                vinputs, vmask, vlabels = vdata
                vinputs = vinputs.to(device)
                vlabels = vlabels.to(device)
                vmask = vmask.to(device)
                voutputs, _ = model(vinputs)
                vloss = torch.mean(criterion(voutputs, vlabels) * vmask)
                correc_predictions += (torch.norm((voutputs - vlabels), dim = 1) < 0.1).sum() # compute accuracy if abs(pred - true) < 0.1
                total_predictions += len(vlabels)
                running_vloss += vloss

        avg_vloss = running_vloss / (i + 1)
        print('LOSS val {} valid {}'.format(avg_vloss, avg_vloss))
        print('LOSS accuracy {}'.format(correc_predictions/total_predictions))
        model.train(True)

        writer.add_scalar("ValEpochLoss", avg_vloss, epoch)

    return train_loss

NUM_DIST = 1

def training_loop():

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(0)

    train_data_loader, val_data_loader = create_dataloader()

    model = PointCloudCollisionPredictor(NUM_DIST, normal_channel=False)
    classifier = model.to(device)
    writer = SummaryWriter()

    criterion = nn.BCELoss(reduction='none')
    # criterion = nn.MSELoss()
    criterion = criterion.to(device)
    optimizer = torch.optim.Adam(
        classifier.parameters(),
        lr=0.005,
        betas=(0.9, 0.999),
        eps=1e-08,
        weight_decay=1e-4
    )

    train(classifier, train_data_loader, val_data_loader, optimizer, criterion, device, writer)

if __name__ == '__main__':
    training_loop()