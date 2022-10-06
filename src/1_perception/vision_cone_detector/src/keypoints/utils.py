'''
Script utils to develope auxiliar functions.

@author: Mariano del Río
@date: 20211204
'''

import math
import csv
import numpy as np
import cv2
from matplotlib import pyplot as PLT
import random as rd


def get_labels(dataset_labels_path, num_images, start, i, j):
    '''
    Function to get labels of a csv file.
    '''
    labels = []
    list_name_images = []

    with open(dataset_labels_path, encoding='utf-8') as f:
        lector = csv.reader(f)
        next(lector)
        for name, _, top, mid_L_top, mid_R_top, mid_L_bot, mid_R_bot, bot_L, bot_R, _ in lector:

            top = getPoints(top)
            mid_L_top = getPoints(mid_L_top)
            mid_R_top = getPoints(mid_R_top)
            mid_L_bot = getPoints(mid_L_bot)
            mid_R_bot = getPoints(mid_R_bot)
            bot_L = getPoints(bot_L)
            bot_R = getPoints(bot_R)

            tuplekp = (top, mid_L_top, mid_R_top, mid_L_bot, mid_R_bot, bot_L, bot_R)
            tuplekp2 = tuplekp[i:j]

            keypointsCono = list(tuplekp2)
            labels.append(keypointsCono)
            list_name_images.append(name)

    zip_rd = [[e0, e1] for e0, e1 in zip(labels, list_name_images)]
    rd.shuffle(zip_rd)
    l0 = [e0 for e0, e1 in zip_rd]
    l1 = [e1 for e0, e1 in zip_rd]
    return np.array(l0[start:num_images]), l1[start:num_images]


def getPoints(string):

    c1 = int(string.split(",")[0][1:])
    c2 = int(string.split(",")[1][:-1])
    coordinates = [c1, c2]
    return np.array(coordinates)


def prep_image(image, target_image_size):

    h, w, _ = image.shape
    image = cv2.resize(image, target_image_size)
    return image


def prep_label(label, target_image_size, orig_image_size, image_path):

    hm = np.zeros((label.shape[0], target_image_size[0], target_image_size[1]))
    for i in range(label.shape[0]):

        row = label[i]
        hm_tmp = np.zeros((orig_image_size[0], orig_image_size[1]))
        hm_tmp[int(row[1]), int(row[0])] = 1.0
        hm[i] = cv2.resize(hm_tmp, target_image_size)
        hm[i] = cv2.GaussianBlur(hm[i], (5, 5), 0)

        if hm[i].sum() == 0:
            print("Incorrect Data Label Detected!")
            print(image_path)

        hm[i] /= hm[i].sum()
    return hm


def get_scale(actual_image_size, target_image_size):
    target_h, target_w = target_image_size
    h_scale = target_h / actual_image_size[0]
    w_scale = target_w / actual_image_size[1]
    return h_scale, w_scale


def scale_labels(labels, h_scale, w_scale):
    new_labels = []
    for pt in np.array(labels):
        x_coor = math.ceil((int(pt[0])) * w_scale)
        y_coor = math.ceil((int(pt[1])) * h_scale)
        new_labels.append([x_coor, y_coor])
    return np.asarray(new_labels)


def unscale_labels(labels, h_scale, w_scale):
    new_labels = []
    for pt in np.array(labels):
        x_coor = math.ceil(pt[0] / w_scale)
        y_coor = math.ceil(pt[1] / h_scale)
        new_labels.append([x_coor, y_coor])
    return np.asarray(new_labels)


def calculate_distance(target_points, pred_points, num_kpt):

    tpt = target_points.view(target_points.shape[0], num_kpt, -1)
    tp = tpt.tolist()
    ppt = pred_points.view(pred_points.shape[0], num_kpt, -1)
    pp = ppt.tolist()
    dist_matrix = []
    for j, points in enumerate(tp):
        list_ = []
        for i, point in enumerate(points):
            dist = np.sqrt(np.square(point[0] - pp[j][i][0])
                           + np.square(point[1] - pp[j][i][1]))
            list_.append(dist)
        dist_matrix.append(list_)
    return dist_matrix


def euclidean_distance_loss(y_pred, Y):

    # Get number of keypoints
    num_kpt = 7

    distances = calculate_distance(Y, y_pred, num_kpt)

    # Calculate mean distance of an image (sum of distances of each keypoint)
    mean = 0
    for d_image in distances:
        mean += sum(d_image)

    return mean / len(distances)


def draw_and_save_graphic(epochs, train_final_loss, val_final_loss, n, pathname):

    '''
    Function to draw graphic of a train of keypoint net. Four graphics are
    drown with train loss, validation loss.
    It removes first n values to avoid noisy data
    '''

    fig = PLT.figure()
    PLT.rcParams.update({'font.size': 6})  # Size of text

    ax1 = fig.add_subplot(221)
    ax1.plot(train_final_loss[n:], label='train_loss')
    ax1.set_xlim(0, epochs+1)
    ax1.set_ylim(0, max(train_final_loss[n:]))
    PLT.legend()

    ax2 = fig.add_subplot(222)
    ax2.plot(val_final_loss[n:], label='validation_loss')
    ax2.set_xlim(0, epochs+1)
    ax2.set_ylim(0, max(val_final_loss[n:]))
    PLT.legend()

    PLT.legend()
    # PLT.show()
    PLT.savefig(pathname)
