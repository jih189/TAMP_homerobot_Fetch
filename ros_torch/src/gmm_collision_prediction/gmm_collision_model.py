import torch.nn as nn
import torch.nn.functional as F
import torch.optim.lr_scheduler as lr_sched


from pointnet2_ops.pointnet2_modules import PointnetSAModule, PointnetSAModuleMSG


def set_bn_momentum_default(bn_momentum):
    def fn(m):
        if isinstance(m, (nn.BatchNorm1d, nn.BatchNorm2d, nn.BatchNorm3d)):
            m.momentum = bn_momentum

    return fn


class BNMomentumScheduler(lr_sched.LambdaLR):
    def __init__(self, model, bn_lambda, last_epoch=-1, setter=set_bn_momentum_default):
        if not isinstance(model, nn.Module):
            raise RuntimeError(
                "Class '{}' is not a PyTorch nn Module".format(type(model)._name_)
            )

        self.model = model
        self.setter = setter
        self.lmbd = bn_lambda

        self.step(last_epoch + 1)
        self.last_epoch = last_epoch

    def step(self, epoch=None):
        if epoch is None:
            epoch = self.last_epoch + 1

        self.last_epoch = epoch
        self.model.apply(self.setter(self.lmbd(epoch)))

    def state_dict(self):
        return dict(last_epoch=self.last_epoch)

    def load_state_dict(self, state):
        self.last_epoch = state["last_epoch"]
        self.step(self.last_epoch)


class PointCloudCollisionPredictor(nn.Module):

    def __init__(self,num_classes):
        super(PointCloudCollisionPredictor, self).__init__()
        self._build_model(num_classes=num_classes)


    def _build_model(self, num_classes: int):
        self.SA_modules = nn.ModuleList()
        self.SA_modules.append(
            PointnetSAModuleMSG(
                npoint=512,
                radii=[0.1, 0.2, 0.4],
                nsamples=[16, 32, 128],
                mlps=[[0, 32, 32, 64], [0, 64, 64, 128], [0, 64, 96, 128]],
                use_xyz=True,
            )
        )

        input_channels = 64 + 128 + 128
        self.SA_modules.append(
            PointnetSAModuleMSG(
                npoint=128,
                radii=[0.2, 0.4, 0.8],
                nsamples=[32, 64, 128],
                mlps=[
                    [input_channels, 64, 64, 128],
                    [input_channels, 128, 128, 256],
                    [input_channels, 128, 128, 256],
                ],
                use_xyz=True,
            )
        )
        self.SA_modules.append(
            PointnetSAModule(
                mlp=[128 + 256 + 256, 256, 512, 1024],
                use_xyz=True,
            )
        )

        self.fc_layer = nn.Sequential(
            nn.Linear(1024, 512, bias=False),
            nn.BatchNorm1d(512),
            nn.ReLU(True),
            nn.Linear(512, 256, bias=False),
            nn.BatchNorm1d(256),
            nn.ReLU(True),
            nn.Dropout(0.5),
            nn.Linear(256, num_classes),
        )

    def _break_up_pc(self, pc):
        xyz = pc[..., 0:3].contiguous()
        features = pc[..., 3:].transpose(1, 2).contiguous() if pc.size(-1) > 3 else None

        return xyz, features

    def forward(self, pointcloud):
        r"""
            Forward pass of the network

            Parameters
            ----------
            pointcloud: Variable(torch.cuda.FloatTensor)
                (B, N, 3 + input_channels) tensor
                Point cloud to run predicts on
                Each point in the point-cloud MUST
                be formated as (x, y, z, features...)
        """
        xyz, features = self._break_up_pc(pointcloud)


        for module in self.SA_modules:
            # print(xyz)
            xyz, features = module(xyz, features)

        # print("XYZ : ", xyz)
        return self.fc_layer(features.squeeze(-1))
        

    def training_step(self, batch, batch_idx):
        pc, _, labels = batch
        # print(pc.dtype)
        # print(labels.dtype)

        logits = self.forward(pc.float())
        # print(logits.shape)
        # print(labels.shape)
        loss = F.binary_cross_entropy(torch.sigmoid(logits), labels)
        with torch.no_grad():
            acc = ((torch.sigmoid(logits) -  labels) ** 2 < 0.0025).float().mean()

        log = dict(train_loss=loss, train_acc=acc)
        return dict(loss=loss, log=log, progress_bar=dict(train_acc=acc))

    def validation_step(self, batch, batch_idx):
        # print(batch)
        pc, _, labels = batch
        # print(pc.dtype, pc.shape)
        # print(labels.dtype)

        logits = self.forward(pc.float())
        # print(logits.shape)
        # print(labels.shape)
        print(torch.sigmoid(logits[4]), labels[4])
        loss = F.binary_cross_entropy(torch.sigmoid(logits), labels)
        acc = ((torch.sigmoid(logits) -  labels) ** 2 < 0.0025).float().mean()

        return dict(val_loss=loss, val_acc=acc)


    def configure_optimizers(self):

        lr_clip = 1e-5
        bnm_clip = 1e-2

        lr_lbmd = lambda _: max(
            0.7
            ** (
                int(
                    self.global_step
                    * 16
                    / 2e4
                )
            ),
            lr_clip / 1e-3,
        )
        bn_lbmd = lambda _: max(
            0.5
            * 0.5
            ** (
                int(
                    self.global_step
                    * 16
                    / 2e4
                )
            ),
            bnm_clip,
        )

        optimizer = torch.optim.Adam(
            self.parameters(),
            lr=1e-3,
            weight_decay=0.0,
        )
        lr_scheduler = lr_sched.LambdaLR(optimizer, lr_lambda=lr_lbmd)
        bnm_scheduler = BNMomentumScheduler(self, bn_lambda=bn_lbmd)

        return [optimizer], [lr_scheduler, bnm_scheduler]


def create_model():
    return PointCloudCollisionPredictor(131).to(device = "cuda:0")

if __name__ == '__main__':
    import  torch
    model = create_model()
    xyz = torch.rand(2, 5000, 3).to(device = "cuda:0")
    print(xyz.dtype, xyz.shape, xyz.device)
    print(model(xyz)[0].shape)