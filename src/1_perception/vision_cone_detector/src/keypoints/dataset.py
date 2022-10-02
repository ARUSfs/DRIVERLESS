# -*- coding: utf-8 -*-

import torch
from torch.utils.data import Dataset
import cv2
from utils import get_labels, prep_image, get_scale, scale_labels


# Class for dataset tagged with labels (To train and validate)
class ConeDataset(Dataset):
    def __init__(self, start, num_images, dataset_images_path, dataset_labels_path,
                 dataset_output_path, target_image_size, num_kpt):

        labels, list_images = get_labels(dataset_labels_path, num_images, start)
        images = list_images
        self.images = images
        self.labels = labels
        self.target_image_size = target_image_size
        self.dataset_path = dataset_images_path
        self.num_keypoints = num_kpt
        self.dataset_output_path = dataset_output_path

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        # Process image
        image = cv2.imread(self.dataset_path+self.images[index])
        orig_image_size = image.shape
        image_name = self.images[index].split(".")[0]
        image = prep_image(image=image, target_image_size=self.target_image_size)

        # Process label
        h_scale, w_scale = get_scale(actual_image_size=orig_image_size,
                                     target_image_size=self.target_image_size)
        scaled_labels = scale_labels(self.labels[index], h_scale, w_scale)
        scaled_labels = scaled_labels / self.target_image_size[0]

        image = image.transpose((2, 0, 1)) / 255.0
        tensor_image = torch.from_numpy(image).type('torch.FloatTensor')
        tensor_labels = torch.from_numpy(scaled_labels).type('torch.FloatTensor')

        return tensor_image, tensor_labels, image_name, orig_image_size


# Class for dataset of any images
class Dataset_for_test(Dataset):
    def __init__(self, names):
        self.images = names
        self.target_image_size = (80, 80)

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        # Image is already process
        image = self.images[index]
        orig_image_size = image.shape
        # image_name = self.images[index].split(".")[0]
        image = prep_image(image=image, target_image_size=self.target_image_size)

        if image.shape[-1] == 3:
            image = image.transpose((2, 0, 1))  # Transpose if matrix is not correct
            image = image / 255.0
            tensor_image = torch.from_numpy(image).type('torch.FloatTensor')

        return tensor_image, orig_image_size
