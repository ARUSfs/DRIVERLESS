"""
Script to develope functions to initialize, train
and evaluate a model

@author: Mariano del Río
@date: 20211204
"""

import numpy as np
from tqdm import tqdm

import torch
from torch.utils.data import DataLoader

from utils import draw_and_save_graphic
from dataset import ConeDataset
from cross_ratio_loss import CrossRatioLoss
from utils import get_scale, scale_labels,unscale_labels



def train_model(kNet, epochs, dataset_images_path,
                dataset_labels_path, num_images_train,
                num_images_validation, target_image_size,
                lr, geo_loss_gamma_horz,
                geo_loss_gamma_vert, num_kpt, i, j,
                batch_train, batch_validation, loss_type, include_geo,device):
    '''
    Function to initialize a model and train it. These are all necessary
    parameters to do it.
    '''
    criterion=CrossRatioLoss(loss_type,
                             include_geo,
                             geo_loss_gamma_horz,
                             geo_loss_gamma_vert)

    #Create datasets
    datasetTrain=ConeDataset(0,num_images_train,
                             dataset_images_path,
                             dataset_labels_path,
                             target_image_size,
                             num_kpt,i,j)

    datasetValidation=ConeDataset(num_images_train,
                                  num_images_train
                                  + num_images_validation,
                                  dataset_images_path,
                                  dataset_labels_path,
                                  target_image_size,
                                  num_kpt,i,j)

    #Create dataloader
    dataloader={ 'Train': DataLoader(datasetTrain,
                                     batch_train,
                                     shuffle=True,
                                     drop_last=True),

            'Validation': DataLoader(datasetValidation,
                                     batch_validation,
                                     shuffle=True,
                                     drop_last=True)
            }

    #Create optimizer for train
    optimizer = torch.optim.Adam(kNet.parameters(), lr=lr)
    scheduler = torch.optim.lr_scheduler.ExponentialLR(optimizer, gamma=0.99)

    #Train
    kNet.zero_grad()

    fit(kNet, dataloader, epochs,
        criterion, scheduler,optimizer,
        num_kpt, device)

def fit(net, dataloader, epochs,
        loss_function, scheduler,
        optimizer, num_kpt, device):

    train_list_loss = []
    val_list_loss = []

    best_val_loss = float('inf')
    best_epoch= 0
    max_tolerance = 10
    tolerance = 0

    for epoch in range(1, epochs+1):
        print(f"EPOCH {epoch}")
        net.train() #Net in train mode
        train_loss, train_acc = [], []
        total_loss = [0,0,0]
        batch_num = 0
        train_process = tqdm(dataloader["Train"])
        for X, Y_hm, Y_points, name, imsize in train_process:

            X = X.to(device)
            Y_hm = Y_hm.to(device)
            Y_points = Y_points.to(device)

            optimizer.zero_grad()

            output = net(X)

            loc_loss, geo_loss, loss = loss_function(output[0],
                                                     output[1],
                                                     Y_hm,
                                                     Y_points)
            loss.backward()
            optimizer.step()

            loc_loss, geo_loss, loss = loc_loss.item(), geo_loss.item(), loss.item()
            train_process.set_description(f"Batch {batch_num}. Location Loss: {round(loc_loss,5)}. Geo Loss: {round(geo_loss,5)}. Total Loss: {round(loss,5)}")

            total_loss[0] += loc_loss
            total_loss[1] += geo_loss
            total_loss[2] += loss
            batch_num += 1
            train_list_loss.append(loss)

        print(f"\tTraining: MSE/Geometric/Total Loss: {round(total_loss[0]/batch_num,10)}/ {round(total_loss[1]/batch_num,10)}/ {round(total_loss[2]/batch_num,10)}")

        scheduler.step()

        net.eval() #Net in evaluation mode
        val_loc_loss, val_geo_loss, val_loss = 0, 0, 0
        with torch.no_grad():
            loss_sums = [0,0,0]
            batch_num=0
            for X, Y_hm, Y_points, name, imsize in dataloader["Validation"]:
                X = X.to(device)
                Y_hm = Y_hm.to(device)
                Y_points = Y_points.to(device)

                output = net(X)
                loc_loss, geo_loss, loss = loss_function(output[0],
                                                         output[1],
                                                         Y_hm,
                                                         Y_points)

                loss_sums[0] += loc_loss.item()
                loss_sums[1] += geo_loss.item()
                loss_sums[2] += loss.item()
                batch_num += 1

            val_loc_loss = loss_sums[0] / batch_num
            val_geo_loss = loss_sums[1] / batch_num
            val_loss = loss_sums[2] / batch_num
            val_list_loss.append(val_loss)

        print(f"\tValidation: MSE/Geometric/Total Loss: {round(val_loc_loss,10)}/ {round(val_geo_loss,10)}/ {round(val_loss,10)}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_epoch = epoch
            tolerance = 0
        else:
            tolerance += 1

        if tolerance >= max_tolerance:
            print(f"Training is stopped due; loss no longer decreases. Epoch {best_epoch} is has the best validation loss.")
            break

    #Train finished, then draw the graphics
    n=0
    pathname="Loss_graphics.png"
    draw_and_save_graphic(epochs, train_list_loss,
                          val_list_loss, n, pathname)

    print("A file with graphics with the improvement \
          of loss and accuracy has been saved in your project")


def evaluate_model(net, dataset, dataloader, device):
    '''
    Function to predict keypoints of dataset of any images choosen.
    Net must be trained.
    Dataloader must be formed with this dataset.
    Device must be between GPU and CPU
    '''
    target_image_size = dataset.target_image_size
    num_keypoints = net.num_kpt
    net.eval() #Net in evaluation mode
    pts2dimgs = []
    for X, imsize in dataloader:

        X = X.to(device)
        y_map, y_pred = net(X)
        pts2d=[]

        for i in range(len(X)):
            #Unscale
            labels = y_pred[i].detach().cpu().numpy()
            h_scale, w_scale = get_scale((imsize[0][i], imsize[1][i]),
                                         target_image_size)
            labels = labels * target_image_size[0]
            unscaled_labels = unscale_labels(labels , h_scale, w_scale)

            #Add keypoints to list
            pts2d.append(unscaled_labels)

            #Add list of keypoints of an image to all
        pts2dimgs.append(pts2d)

    return pts2dimgs
