# -*- coding: utf-8 -*-

# Importations
import cv2
from utils import get_scale, unscale_labels
import torch


# Detection of a set of images of train set. Parameter draw_cones is in order
# to draw keypoints in cones
def detect_keypoints_from_dataset(net, dataset, dataloader, draw_cones=False, output_path=""):
    target_image_size = dataset.target_image_size
    num_keypoints = dataset.num_keypoints
    dataset_images_path = dataset.dataset_path

    net.eval()  # Net in evaluation mode
    pts2dimgs = []
    for X, Y, name, imsize in dataloader["Test"]:

        X = X.to('cuda' if torch.cuda.is_available() else 'cpu')
        y_pred = net(X)
        y_pred = y_pred.view(y_pred.shape[0], num_keypoints, -1)
        pts2d = []
        # For each image of the batch given by dataloader
        for i in range(len(X)):
            # Unscale
            labels = y_pred[i].detach().cpu().numpy()
            h_scale, w_scale = get_scale((imsize[0][i], imsize[1][i]), target_image_size)
            labels = labels * target_image_size[0]
            unscaled_labels = unscale_labels(labels, h_scale, w_scale)

            # Add keypoints to list
            pts2d.append((name[i], unscaled_labels))

            if draw_cones:
                # Add keypoints predicted to the image
                img = cv2.imread(dataset_images_path+name[i]+".jpg")

                point_size = 1
                point_color1 = (0, 0, 255)
                thickness = 1

                points_list = unscaled_labels
                for point in points_list:
                    cv2.circle(img, point, point_size, point_color1, thickness)

                # Save the image with keypoints
                cv2.imwrite(output_path+name[i]+".jpg", img)

        # Add list of keypoints of an image to all
        pts2dimgs.append(pts2d)

    return pts2dimgs


# Detection of a set of any images. Parameter draw_cones is in order to draw keypoints in cones.
# Images path indicate the path of the images to predict its keypoints
def detect_keypoints_from_set_images(net, dataset, dataloader):
    target_image_size = dataset.target_image_size
    num_keypoints = net.num_kpt
    net.eval()  # Net in evaluation mode
    pts2dimgs = []
    for X, imsize in dataloader:

        X = X.to('cuda' if torch.cuda.is_available() else 'cpu')
        y_pred = net(X)
        y_pred = y_pred.view(y_pred.shape[0], num_keypoints, -1)
        pts2d = []
        # For each image of the batch given by dataloader
        for i in range(len(X)):
            # Unscale
            labels = y_pred[i].detach().cpu().numpy()
            h_scale, w_scale = get_scale((imsize[0][i], imsize[1][i]), target_image_size)
            labels = labels * target_image_size[0]
            unscaled_labels = unscale_labels(labels, h_scale, w_scale)

            # Add keypoints to list
            pts2d.append(unscaled_labels)

            # Add list of keypoints of an image to all
        pts2dimgs.append(pts2d)

    return pts2dimgs
