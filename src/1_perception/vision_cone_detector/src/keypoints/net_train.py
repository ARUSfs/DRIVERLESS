# -*- coding: utf-8 -*-


import torch
import numpy as np
from torch.optim.lr_scheduler import StepLR
from dataset import ConeDataset
from utils import measure_accuracy, draw_and_save_graphic
from torch.utils.data import DataLoader


# Net train function: Net is trained and return graphic with the improvements of loss and accuracy
def fit(net, dataloader, epochs, error, criterion, scheduler):
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    train_final_loss = []
    train_final_acc = []
    val_final_loss = []
    val_final_acc = []

    for epoch in range(1, epochs+1):

        net.train()  # Net in train mode
        train_loss, train_acc = [], []
        for X, YY, name, imsize in dataloader["Train"]:
            X = X.to(device)
            scheduler.optimizer.zero_grad()

            Y = YY.view(YY.shape[0], -1).to(device)  # Reshape from [X, 7, 2] to [X, 14] to Net
            y_pred = net(X).to(device)

            loss = criterion(y_pred, Y)

            loss.backward()

            scheduler.optimizer.step()

            train_loss.append(loss.item())

            acc = measure_accuracy(y_pred, Y, error)
            train_acc.append(acc)

        val_loss, val_acc = [], []

        scheduler.step()
        # Net validation
        net.eval()  # Net in evaluation mode

        with torch.no_grad():
            for X, YY, name, imsize in dataloader["Validation"]:

                X = X.to(device)
                Y = YY.view(YY.shape[0], -1).to(device)  # Reshape from [X, 7, 2] to [X, 14] to Net
                y_pred = net(X).to(device)
                loss = criterion(y_pred, Y)
                val_loss.append(loss.item())
                acc = measure_accuracy(y_pred, Y, error)
                val_acc.append(acc)

        train_final_loss.append(np.mean(train_loss))
        train_final_acc.append(np.mean(train_acc))
        val_final_loss.append(np.mean(val_loss))
        val_final_acc.append(np.mean(val_acc))

        if epoch % 10 == 0:
            # Print every 10 epochs
            print("Epoch", epoch, "/", epochs)
            print("Train_loss =>", np.mean(train_loss), " // Train_acc=>", np.mean(train_acc))
            print("Val_loss=>", np.mean(val_loss), "// Val_acc=>", np.mean(val_acc), "\n")

    # Train finished, then draw the graphics
    n = 20
    if epochs < n:
        n = epochs-1
    pathname = "Train_accuracy_grahics.png"
    draw_and_save_graphic(epochs, train_final_loss, train_final_acc,
                          val_final_loss, val_final_acc, n, pathname)
    print("A file with graphics with the improvement of loss and accuracy has been \
          saved in your project")


# Net train function for hiperparameters. Instead of draw graphics,
# return values to draw with others trains
def fit_hiperparameters(net, dataloader, epochs, error, criterion, scheduler):
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    train_final_loss = []
    train_final_acc = []
    val_final_loss = []
    val_final_acc = []

    for epoch in range(1, epochs+1):
        net.train()  # Model in train mode
        train_loss, train_acc = [], []
        for X, YY, name, imsize in dataloader["Train"]:
            X = X.to(device)
            scheduler.optimizer.zero_grad()

            Y = YY.view(YY.shape[0], -1).to(device)  # Reshape from [X, 7, 2] to [X, 14] to Net
            y_pred = net(X).to(device)
            loss = criterion(y_pred, Y)

            loss.backward()

            scheduler.optimizer.step()

            train_loss.append(loss.item())

            acc = measure_accuracy(y_pred, Y, error)
            train_acc.append(acc)

            val_loss, val_acc = [], []

        scheduler.step()
        # Net validation
        net.eval()

        with torch.no_grad():  # Without gradients change
            for X, YY, name, imsize in dataloader["Validation"]:

                X = X.to(device)
                Y = YY.view(YY.shape[0], -1).to(device)  # Reshape from [X, 7, 2] to [X, 14] to Net
                y_pred = net(X).to(device)
                loss = criterion(y_pred, Y)
                val_loss.append(loss.item())
                acc = measure_accuracy(y_pred, Y, error)
                val_acc.append(acc)

        train_final_loss.append(np.mean(train_loss))
        train_final_acc.append(np.mean(train_acc))
        val_final_loss.append(np.mean(val_loss))
        val_final_acc.append(np.mean(val_acc))

        if epoch % 10 == 0:
            # Print every 10 epochs
            print("Epoch", epoch, "/", epochs)
            print("Train_loss =>", np.mean(train_loss), " // Train_acc=>", np.mean(train_acc))
            print("Val_loss=>", np.mean(val_loss), "// Val_acc=>", np.mean(val_acc), "\n")

    # Quit first n values to a better visualization of graphic
    n = 30
    return train_final_loss[n:], train_final_acc[n:], val_final_loss[n:], val_final_acc[n:]


def train_model(kNet, epochs, dataset_images_path, dataset_labels_path,
                num_images_train, num_images_validation):
    lrRes = 0.075
    lrOut = 0.175
    points_error = 0.02
    criterion = torch.nn.MSELoss()
    optimizer = torch.optim.Adam
    target_image_size = (80, 80)
    num_kpt = 7
    batch_train = 64
    batch_validation = 64
    output_path = ""
    # Create datasets for train
    datasetTrain = ConeDataset(0, num_images_train, dataset_images_path, dataset_labels_path,
                               output_path, target_image_size, num_kpt)

    datasetValidation = ConeDataset(num_images_train, num_images_train + num_images_validation,
                                    dataset_images_path, dataset_labels_path, output_path,
                                    target_image_size, num_kpt)

    # Create dataloader for train
    dataloader = {'Train': DataLoader(datasetTrain, batch_train, shuffle=True, drop_last=True),
                  'Validation': DataLoader(datasetValidation, batch_validation,
                                           shuffle=True, drop_last=True)}

    # Create optimizer for train
    optimizerr = optimizer([{'params': kNet.resNet.parameters(), 'lr': lrRes},
                            {'params': kNet.out.parameters(), 'lr': lrOut}])

    scheduler = StepLR(optimizerr, step_size=100, gamma=0.1)
    # Train
    kNet.zero_grad()
    fit(kNet, dataloader, epochs, points_error, criterion, scheduler)
    # Save model
    torch.save(kNet.state_dict(), "train_34.pth")
