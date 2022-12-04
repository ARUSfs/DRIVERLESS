"""
Script to optimize hiperparameters of keypoint
neural net by bayesian optimization.

@author: Mariano del Río
@date: 20211204
"""

import numpy as np
import torch

from net_architecture import KeypointNet
from utils import euclidean_distance_loss
from cross_ratio_loss import CrossRatioLoss
from dataset import ConeDataset

from ax.service.managed_loop import optimize
from torch.utils.data import DataLoader


def fit_hiperparameters(net, dataloader, epochs, loss_function,
                        optimizer, scheduler, device):

    for epoch in range(1, epochs+1):
        net.train()  # Model in train mode
        for X, Y_hm, Y_points, name, imsize in dataloader["Train"]:
            X = X.to(device)
            Y_hm = Y_hm.to(device)
            Y_points = Y_points.to(device)
            optimizer.zero_grad()
            output = net(X)
            loc_loss, geo_loss, loss = loss_function(output[0], output[1],
                                                     Y_hm, Y_points)
            loss.backward()
            optimizer.step()

        if epoch % 100 == 0:
            print("Epoch -> "+str(epoch))

        scheduler.step()


def evaluate(net, dataloader, num_kpt, device):

    val_acc = []

    net.eval()  # Net in evaluation mode
    with torch.no_grad():
        for X, Y_hm, Y_points, name, imsize in dataloader["Validation"]:
            X = X.to(device)
            Y_hm = Y_hm.to(device)
            Y_points = Y_points.to(device)
            output = net(X)  # 0 is heat map and 1 points
            acc = euclidean_distance_loss(output[1], Y_points)
            val_acc.append(acc)

    return np.mean(val_acc)


def train_evaluate(parameters):
    """
    Main function to pass to bayesian optimization API
    Create a model, train and evaluate it.
    """

    # Create neural net
    num_kpt = 7
    target_image_size = (80, 80)
    kNet = KeypointNet(target_image_size, num_kpt)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    kNet = kNet.to(device)

    # Parameters
    epochs = 4
    dataset_images_path = "Dataset/Images/RektNet_Dataset/"
    dataset_labels_path = "Dataset/Labels/rektnet_labels.csv"
    num_images_train = 16
    num_images_validation = 16
    first_kp = 0
    last_kp = 7
    batch_train = 16
    batch_validation = 16
    loss_type = "l1_softargmax"
    include_geo = True

    lr = parameters["lr"]
    geo_loss_gamma_horz = parameters["geo_loss_gamma_horz"]
    geo_loss_gamma_vert = parameters["geo_loss_gamma_vert"]
    gamma = parameters["gamma"]

    loss_function = CrossRatioLoss(loss_type, include_geo,
                                   geo_loss_gamma_horz, geo_loss_gamma_vert)

    # Create datasets
    datasetTrain = ConeDataset(0, num_images_train, dataset_images_path,
                               dataset_labels_path, target_image_size,
                               num_kpt, first_kp, last_kp)

    datasetValidation = ConeDataset(num_images_train,
                                    num_images_train + num_images_validation,
                                    dataset_images_path, dataset_labels_path,
                                    target_image_size, num_kpt, first_kp, last_kp)

    # Create dataloader
    dataloader = {'Train': DataLoader(datasetTrain, batch_train,
                                      shuffle=True, drop_last=True),
                  'Validation': DataLoader(datasetValidation, batch_validation,
                                           shuffle=True, drop_last=True)}

    # Create optimizer for train
    optimizer = torch.optim.Adam(kNet.parameters(), lr=lr)
    scheduler = torch.optim.lr_scheduler.ExponentialLR(optimizer, gamma)

    # Train
    kNet.zero_grad()
    fit_hiperparameters(kNet, dataloader, epochs, loss_function,
                        optimizer, scheduler, device)
    accuracy = evaluate(kNet, dataloader, num_kpt, device)

    print("Euclidean loss -> ", accuracy)

    return accuracy


def optimize_param():
    '''
    Function to optimize keypoint detector. We choose these hiperparameters
    and its range or values and the method optimize neural net.
    '''
    parameterization = [
        {"name": "lr", "type": "range", "bounds": [1e-6, 0.1]},
        {"name": "geo_loss_gamma_horz", "type": "range", "bounds": [1e-8, 0.1]},
        {"name": "geo_loss_gamma_vert", "type": "range", "bounds": [1e-8, 0.1]},
        {"name": "gamma", "type": "range", "bounds": [1e-8, 1.0]},
        ]

    best_parameters, values, experiment, model = optimize(
            parameters=parameterization,
            evaluation_function=train_evaluate,
            minimize=True,
            total_trials=200)

    print(best_parameters)


def main():

    optimize_param()


if __name__ == "__main__":
    main()
