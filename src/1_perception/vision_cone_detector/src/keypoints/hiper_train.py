# Script to optimize hiperparameters like learning
# rates and epochs

import torch
from torch.utils.data import DataLoader
from torch.optim.lr_scheduler import StepLR
from net_architecture import KeypointNet
from net_train import fit_hiperparameters
from dataset import ConeDataset
from utils import draw_and_save_graphics


# Function to contruct and train a model with certain parameters
def model_hiper(i, dataset_images_path, dataset_labels_path, dataset_output_path,
                num_images_train, num_images_validation, num_images_test, epochs,
                lrRes, lrOut, error, criterion, optimizer, target_image_size,
                num_kpt, batch_train, batch_validation, batch_test):
    # Create net
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    kNet = KeypointNet(target_image_size, num_kpt).to(device)
    kNet.zero_grad()

    # Create datasets
    datasetTrain = ConeDataset(0, num_images_train, dataset_images_path, dataset_labels_path,
                               dataset_output_path, target_image_size, num_kpt)

    datasetValidation = ConeDataset(num_images_train, num_images_train + num_images_validation,
                                    dataset_images_path, dataset_labels_path, dataset_output_path,
                                    target_image_size, num_kpt)

    datasetTest = ConeDataset(num_images_validation, num_images_validation + num_images_test,
                              dataset_images_path, dataset_labels_path, dataset_output_path,
                              target_image_size, num_kpt)

    # Create dataloader
    dataloader = {'Train': DataLoader(datasetTrain, batch_train, shuffle=True, drop_last=True),
                  'Validation': DataLoader(datasetValidation, batch_validation, shuffle=True,
                                           drop_last=True),
                  'Test': DataLoader(datasetTest, batch_test)}
    # Create optimizer
    optimizerr = optimizer([{'params': kNet.resNet.parameters(), 'lr': lrRes},
                            {'params': kNet.out.parameters(), 'lr': lrOut}])
    scheduler = StepLR(optimizerr, step_size=110, gamma=0.1)

    # Call fit function
    print("Let's train")
    tl, ta, vl, va = fit_hiperparameters(kNet, dataloader, epochs, error, criterion, scheduler)

    # Save model
    torch.save(kNet.state_dict(), "Train-model-"+str(i)+".pth")
    return tl, ta, vl, va


# Function to optimize parameters: 4 models for diferent parameters are trained and we can
# see graphics and data
def optimize_hiper(epochs, dataset_images_path, dataset_labels_path,
                   dataset_output_path, num_images_train,
                   num_images_validation, num_images_test,
                   error, criterion, optimizer, target_image_size, num_kpt,
                   batch_train, batch_validation, batch_test):
    # Parameters to optimize: lrRes, lrOut

    print("First model")
    lrRes1 = 0.075
    lrOut1 = 0.125

    tl1, ta1, vl1, va1 = model_hiper(1, dataset_images_path, dataset_labels_path,
                                     dataset_output_path, num_images_train,
                                     num_images_validation, num_images_test,
                                     epochs, lrRes1, lrOut1, error, criterion,
                                     optimizer, target_image_size, num_kpt,
                                     batch_train, batch_validation, batch_test)

    print("2 model")
    lrRes2 = 0.05
    lrOut2 = 0.1

    tl2, ta2, vl2, va2 = model_hiper(2, dataset_images_path, dataset_labels_path,
                                     dataset_output_path, num_images_train,
                                     num_images_validation, num_images_test,
                                     epochs, lrRes2, lrOut2, error, criterion,
                                     optimizer, target_image_size, num_kpt,
                                     batch_train, batch_validation, batch_test)

    print("3 model")
    lrRes3 = 0.1
    lrOut3 = 0.125
    tl3, ta3, vl3, va3 = model_hiper(3, dataset_images_path, dataset_labels_path,
                                     dataset_output_path, num_images_train,
                                     num_images_validation, num_images_test,
                                     epochs, lrRes3, lrOut3, error, criterion,
                                     optimizer, target_image_size, num_kpt,
                                     batch_train, batch_validation, batch_test)

    print("4 model")
    lrRes4 = 0.02
    lrOut4 = 0.05

    tl4, ta4, vl4, va4 = model_hiper(4, dataset_images_path, dataset_labels_path,
                                     dataset_output_path, num_images_train,
                                     num_images_validation, num_images_test,
                                     epochs, lrRes4, lrOut4, error, criterion,
                                     optimizer, target_image_size, num_kpt,
                                     batch_train, batch_validation, batch_test)

    # Create four graphics to train_loss, val_loss, train_acc
    # and val_acc of each model
    path_name = "TrainHiperparametersGraphics.png"

    draw_and_save_graphics(tl1, tl2, tl3, tl4, ta1, ta2, ta3, ta4, vl1, vl2, vl3, vl4, va1,
                           va2, va3, va4, lrRes1, lrOut1, lrRes2, lrOut2, lrRes3, lrOut3,
                           lrRes4, lrOut4, epochs, path_name)


def main():

    print("Optimization of hiperparameters")

    dataset_images_path = "Dataset/Images/RektNet_Dataset/"
    dataset_labels_path = "Dataset/Labels/rektnet_labels.csv"
    output_path = "Dataset/Images/Output/"
    num_images_train = 2950
    num_images_validation = 200
    num_images_test = 40
    epochs = 300
    points_error = 0.02
    loss_function = torch.nn.MSELoss()
    optimizer = torch.optim.Adam
    target_image_size = (80, 80)
    num_kpt = 7
    batch_train = 32
    batch_validation = 32
    batch_test = 32

    optimize_hiper(epochs, dataset_images_path, dataset_labels_path,
                   output_path, num_images_train,
                   num_images_validation, num_images_test,
                   points_error, loss_function, optimizer,
                   target_image_size, num_kpt, batch_train,
                   batch_validation, batch_test)


if __name__ == "__main__":
    main()
