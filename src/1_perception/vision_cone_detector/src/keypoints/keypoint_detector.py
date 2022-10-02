import numpy as np
import cv2
import torch
from torch.utils.data import DataLoader
from net_architecture import KeypointNet
from net_train import train_model
from evaluation import detect_keypoints_from_set_images
from dataset import Dataset_for_test


class keypoint_detector():

    def __init__(self, target_image_size=(80, 80), num_kpt=7):
        '''
        Initialization of a keypoints detector.
        '''
        self.kNet = KeypointNet(target_image_size, num_kpt)
        # self.kNet.load_state_dict(torch.load("train.pth"))
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.kNet = self.kNet.to(self.device)

    def train(self):
        '''
        Train model with images and labels.
        Train with number of images indicated in num_images_train
        and num_images_validation in every epoch.
        '''
        epochs = 300
        dataset_images_path = "Dataset/Images/RektNet_Dataset/"
        dataset_labels_path = "Dataset/Labels/rektnet_labels.csv"
        num_images_train = 2950
        num_images_validation = 200

        train_model(self.kNet, epochs, dataset_images_path, dataset_labels_path,
                    num_images_train, num_images_validation)

    def get_keypoints_from_images(self, test_images):
        '''
        Method to get keypoints from a set of images.
        Images must be a numpy array of images read by openCv.
        This returns a numpy array of arrays of keypoints
        '''
        datasetTest = Dataset_for_test(test_images)
        dataloaderTest = DataLoader(datasetTest)

        pts2d = detect_keypoints_from_set_images(self.kNet, datasetTest, dataloaderTest)
        pts2d = np.array([cone[0] for cone in pts2d])

        return pts2d

    def get_keypoint_from_image(self, image):
        '''
        Method to predict keypoints of an image.
        This image must be an array of the image read by openCV.
        This returns a numpy array of keypoints
        '''

        images = [image]
        datasetTest = Dataset_for_test(images)
        dataloaderTest = DataLoader(datasetTest)

        pts2d = detect_keypoints_from_set_images(self.kNet, datasetTest, dataloaderTest)
        pts2d = pts2d[0][0]
        return pts2d

    def show_keypoint_image(self, image, keypoints, name):
        '''
        Method to show keypoints drown in image.
        Image must be an array of the image read by openCV.
        Keypoints must be a list of points (like the output of previous methods)
        '''

        point_size = 2
        point_color1 = (0, 0, 255)
        thickness = 1

        points_list = keypoints
        for point in points_list:
            cv2.circle(image, point, point_size, point_color1, thickness)

        # Save the image with keypoints
        cv2.imwrite(name + "_test", image)
        print("Image has been saved in this project with name :"+name)


def main():
    detector = keypoint_detector()
    detector.train()
    # image = "Dataset/vid_5_frame_1196_0.jpg"
    # img = cv2.imread(image)

    # pts = detector.get_keypoint_from_image(img)

    # detector.show_keypoint_image(img, pts, image)


if __name__ == "__main__":
    main()
