"""
Script to implement dataset class for train

@author: Mariano del Río
@date: 20211204

"""

import torch
from torch.utils.data import Dataset
from torchvision import transforms

import numpy as np
import cv2
import random
import math

from utils import get_labels, get_scale, scale_labels, prep_image, prep_label

class ConeDataset(Dataset):

    def __init__(self, start,num_images,
                 dataset_images_path,
                 dataset_labels_path,
                 target_image_size, num_kpt,i,j):

        labels , images = get_labels(dataset_labels_path,num_images,start,i,j)
        self.images = images
        self.labels = labels
        self.target_image_size = target_image_size
        self.dataset_path = dataset_images_path
        self.num_keypoints=num_kpt

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):

        image = cv2.imread(self.dataset_path+self.images[index])
        orig_image_size = image.shape
        image_name = self.images[index].split(".")[0]
        image = prep_image(image=image,
                           target_image_size=self.target_image_size)

        hm = prep_label(label=self.labels[index],
                        target_image_size=self.target_image_size,
                        orig_image_size=orig_image_size,
                        image_path=self.images[index])

        h_scale, w_scale = get_scale(actual_image_size=orig_image_size,
                                     target_image_size=self.target_image_size)

        scaled_labels = scale_labels(self.labels[index], h_scale, w_scale)
        scaled_labels = scaled_labels / self.target_image_size[0]

        image = image.transpose((2, 0, 1)) / 255.0
        tensor_image = torch.from_numpy(image).type('torch.FloatTensor')
        heat_map=torch.from_numpy(hm).type('torch.FloatTensor')
        tensor_labels=torch.from_numpy(scaled_labels).type('torch.FloatTensor')

        return tensor_image,heat_map,tensor_labels,image_name, orig_image_size

class Dataset_for_test(Dataset):
    """
    Dataset used to test images no labeled.
    Parameters:
        -names: Paths of images to detect.
        -target_image_size: Size of image to reduce all
    """
    def __init__(self,names,target_image_size):
        self.images=names
        self.target_image_size=target_image_size
    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):

        #Image is already process
        image = self.images[index]
        orig_image_size = image.shape
        image = prep_image(image=image,target_image_size=self.target_image_size)

        if image.shape[-1]==3:
            image = image.transpose((2, 0, 1)) #Transpose if matrix is not correct
        image=image/ 255.0
        tensor_image = torch.from_numpy(image).type('torch.FloatTensor')
        return tensor_image, orig_image_size

