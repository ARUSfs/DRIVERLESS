"""
Script to implement a keypoint detector class to
train a neural net and detect keypoints of images.

@author: Mariano del Río
@date: 20211204
"""

import argparse

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader

from .net_train import train_model, evaluate_model
from .dataset import Dataset_for_test
from .net_architecture import KeypointNet


class keypoint_detector():

    def __init__(self, num_kpt, path_weights, target_image_size=(80, 80)):
        '''
        Initialization of a keypoints detector.
        '''

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.kNet = KeypointNet(target_image_size, num_kpt).to(self.device)
        if path_weights is not None:
            self.kNet.load_state_dict(torch.load(path_weights, map_location=self.device))

        self.target_image_size = target_image_size
        self.num_kpt = num_kpt

        print("Net ready!")

    def train(self, epochs, dataset_images_path, dataset_labels_path,
              include_geo):
        '''
        Train model with images and labels.
        These are all parameters to build, train and validate
        the model.
        '''
        num_images_train = 36
        num_images_validation = 36
        lr = 0.1
        geo_loss_gamma_horz = 0.0
        geo_loss_gamma_vert = 0.0
        batch_train = 6
        batch_validation = 6
        loss_type = 'l1_softargmax'
        first_kp, last_kp = 0, self.num_kpt  # Range of keypoints

        if last_kp != 7:
            print("Not allowed paralelism loss")
            include_geo = False

        print("Train!")
        train_model(self.kNet, epochs, dataset_images_path,
                    dataset_labels_path, num_images_train,
                    num_images_validation, self.target_image_size,
                    lr, geo_loss_gamma_horz, geo_loss_gamma_vert,
                    self.num_kpt, first_kp, last_kp, batch_train,
                    batch_validation, loss_type, include_geo, self.device)

        # Save model
        train_file_name = "train.pth"
        torch.save(self.kNet.state_dict(), train_file_name)

    def get_keypoints_from_images(self, test_images: np.ndarray):
        '''
        Method to get keypoints from a set of images.
        Images must be a numpy array of images read by openCV.
        This returns a numpy array of arrays of keypoints
        '''
        datasetTest = Dataset_for_test(test_images, self.target_image_size)
        dataloaderTest = DataLoader(datasetTest, len(test_images))

        pts2d = evaluate_model(self.kNet, datasetTest, dataloaderTest,
                               self.device)

        pts2d = np.array([cone[0] for cone in pts2d])

        print("Evaluated!")
        return pts2d

    def get_keypoint_from_image(self, image: np.ndarray):
        '''
        Method to predict keypoints of an image.
        This image must be an array of the image read by openCV.
        This returns a numpy array of keypoints
        '''

        images = [image]
        datasetTest = Dataset_for_test(images, self.target_image_size)
        dataloaderTest = DataLoader(datasetTest)

        pts2d = evaluate_model(self.kNet, datasetTest, dataloaderTest,
                               self.device)
        pts2d = pts2d[0][0]

        print("Evaluated!")
        return pts2d

    def show_keypoint_image(self, image: np.ndarray, keypoints: np.ndarray, name):
        '''
        Method to show keypoints drown in image.
        Name is the path of the generated image to save
        '''

        point_size = 1
        point_color1 = (0, 0, 255)
        thickness = 1

        for point in keypoints:
            cv2.circle(image, point, point_size, point_color1, thickness)

        # Save the image with keypoints
        cv2.imwrite(name, image)
        print("Image has been saved in this project with name: " + name)


def main(images_path, labels_path, output_path, epochs, draw_keypoints,
         include_geo, num_kpt, path_image, train, path_weights):

    if draw_keypoints and path_image is None:
        print("Insert path image to detect keypoints or \
              modify draw_keypoints to False to not evaluate")
    else:

        detector = keypoint_detector(num_kpt, path_weights)

        if train:
            detector.train(epochs, images_path, labels_path, include_geo)

        if path_image is not None:
            img = cv2.imread(path_image)
            pts = detector.get_keypoint_from_image(img)
            name = path_image.split("/")[-1]
            out = output_path + "detected_" + name
            detector.show_keypoint_image(img, pts, out)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Detection of keypoints:\
                                     Train and evaluate a model')

    parser.add_argument('--epochs', default=1, help='Epochs to train the model.', type=int)
    parser.add_argument('--dataset_images_path', default="Dataset/Images/RektNet_Dataset/",
                        help='Insert dataset images path')
    parser.add_argument('--dataset_labels_path', default="Dataset/Labels/rektnet_labels.csv",
                        help='Insert dataset labels path')
    parser.add_argument('--draw_keypoints', default=True,
                        help='True by default if you want to draw keypoints')
    parser.add_argument('--output_path', default="",
                        help='Insert path for output images if you want to draw keypoints')
    parser.add_argument('--include_geo', default=False,
                        help='True if add paralelism to loss')
    parser.add_argument('--num_kpt', default=7, help='Number of keypoints to detect')
    parser.add_argument('--path_image', default=None, help='Path of image to detect keypoints')
    parser.add_argument('--train', default=True, help='True if you want to train ')
    parser.add_argument('--path_weights', default=None,
                        help='Path of weights. By default train with initial weights')
    args = parser.parse_args()

    main(args.dataset_images_path, args.dataset_labels_path, args.output_path,
         args.epochs, args.draw_keypoints, args.include_geo, args.num_kpt,
         args.path_image, args.train, args.path_weights)
