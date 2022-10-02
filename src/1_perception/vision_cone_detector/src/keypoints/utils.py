# Utils to use in other files

import math
import csv
import numpy as np
import cv2
from matplotlib import pyplot as PLT
import random as rd


# Function to get the labels from a csv
def get_labels(dataset_labels_path, num_images, start):
    labels = []
    list_name_images = []  # We use this list to obtain the images according to its labels
    with open(dataset_labels_path, encoding='utf-8') as f:
        lector = csv.reader(f)
        next(lector)
        for name, _, top, mid_L_top, mid_R_top, mid_L_bot, mid_R_bot, bot_L, bot_R, _ in lector:
            # Hemos añadido un parámetro más, n, porque en csv filas acaban en coma
            top = getPoints(top)
            mid_L_top = getPoints(mid_L_top)
            mid_R_top = getPoints(mid_R_top)
            mid_L_bot = getPoints(mid_L_bot)
            mid_R_bot = getPoints(mid_R_bot)
            bot_L = getPoints(bot_L)
            bot_R = getPoints(bot_R)

            tuple = (top, mid_L_top, mid_R_top, mid_L_bot, mid_R_bot, bot_L, bot_R)
            keypointsCono = list(tuple)
            labels.append(keypointsCono)
            list_name_images.append(name)

    zip_rd = [[e0, e1] for e0, e1 in zip(labels, list_name_images)]
    rd.shuffle(zip_rd)
    l0 = [e0 for e0, e1 in zip_rd]
    l1 = [e1 for e0, e1 in zip_rd]
    return np.array(l0[start:num_images]), l1[start:num_images]


# Function to get points from a string
def getPoints(string):
    # Separate the coordenates
    c1 = int(string.split(",")[0][1:])
    c2 = int(string.split(",")[1][:-1])
    coordinates = [c1, c2]
    return np.array(coordinates)


# Function to resize an image
def prep_image(image, target_image_size):
    h, w, _ = image.shape
    image = cv2.resize(image, target_image_size)
    return image


# Function to resize labels
def prep_label(label, target_image_size, orig_image_size, image_path):
    hm = np.zeros((label.shape[0], target_image_size[0], target_image_size[1]))
    for i in range(label.shape[0]):
        row = label[i]
        # padded_image_size = max(orig_image_size[0], orig_image_size[1])
        hm_tmp = np.zeros((orig_image_size[0], orig_image_size[1]))
        hm_tmp[int(row[1]), int(row[0])] = 1.0
        hm[i] = cv2.resize(hm_tmp, target_image_size)
        hm[i] = cv2.GaussianBlur(hm[i], (5, 5), 0)
        if hm[i].sum() == 0:
            print("Incorrect Data Label Detected! Please revise the image label below \
                  and becoming the one with data!")
            print(image_path)
        hm[i] /= hm[i].sum()
    return hm


# Function to get the scale of an image
def get_scale(actual_image_size, target_image_size):
    # since we are dealing with square image only, is doesn't matter we use height or width
    target_h, target_w = target_image_size
    h_scale = target_h / actual_image_size[0]
    w_scale = target_w / actual_image_size[1]
    return h_scale, w_scale


# Function to scale labels
def scale_labels(labels, h_scale, w_scale):
    new_labels = []
    for pt in np.array(labels):
        x_coor = math.ceil((int(pt[0])) * w_scale)
        y_coor = math.ceil((int(pt[1])) * h_scale)
        new_labels.append([x_coor, y_coor])
    return np.asarray(new_labels)


# Function to unscale labels
def unscale_labels(labels, h_scale, w_scale):
    new_labels = []
    for pt in np.array(labels):
        x_coor = math.ceil(pt[0] / w_scale)
        y_coor = math.ceil(pt[1] / h_scale)
        new_labels.append([x_coor, y_coor])
    return np.asarray(new_labels)


# Function to calculate distance between two list of points
def calculate_distance(target_points, pred_points):
    # We have tensors, so we make transormations to lists
    tpt = target_points.view(target_points.shape[0], 7, -1)
    tp = tpt.tolist()

    ppt = pred_points.view(pred_points.shape[0], 7, -1)
    pp = ppt.tolist()
    dist_matrix = []
    for j, points in enumerate(tp):
        list = []
        for i, point in enumerate(points):
            dist = np.sqrt(np.square(point[0] - pp[j][i][0]) + np.square(point[1] - pp[j][i][1]))
            list.append(dist)
        dist_matrix.append(list)
    return dist_matrix


# Function to measure the accurracy of a trained model with certain error
def measure_accuracy(y_pred, Y, error):
    # A prediction is correct if all distances between points from an image are
    # less than certain error
    distances = calculate_distance(Y, y_pred)
    correct_predictions = sum(1 for d_image in distances if all(d <= error for d in d_image))
    return correct_predictions / y_pred.shape[0]


# Draw and save graphics from train (Four graphics of one parameter:
# Train loss, train accuracy, validation loss, validation accuracy)
def draw_and_save_graphic(epochs, train_final_loss, train_final_acc, val_final_loss,
                          val_final_acc, n, pathname):
    # Quit n first values of lists to see better the decrement of graphics

    fig = PLT.figure()
    PLT.rcParams.update({'font.size': 6})  # Size of text

    ax1 = fig.add_subplot(221)
    ax1.plot(train_final_loss[n:], label='train_loss.')
    ax1.set_xlim(0, epochs+1)
    ax1.set_ylim(0, max(train_final_loss[n:]))
    PLT.legend()

    ax2 = fig.add_subplot(222)
    ax2.plot(train_final_acc[n:], label='train_acc')
    ax2.set_xlim(0, epochs+1)
    ax2.set_ylim(0, max(train_final_acc[n:]))
    PLT.legend()

    ax3 = fig.add_subplot(223)
    ax3.plot(val_final_loss[n:], label='validation_loss')
    ax3.set_xlim(0, epochs+1)
    ax3.set_ylim(0, max(val_final_loss[n:]))
    PLT.legend()

    ax4 = fig.add_subplot(224)
    ax4.plot(val_final_acc[n:], label='validation_acc')
    ax4.set_xlim(0, epochs+1)
    ax4.set_ylim(0, max(val_final_acc[n:]))
    PLT.legend()
    # PLT.show()
    PLT.savefig(pathname)


# Draw and save graphics from train hiperparameters (Four graphics of four parameter:
# Train loss,htrain accuracy, validation loss, validation accuracy)
def draw_and_save_graphics(tl1, tl2, tl3, tl4, ta1, ta2, ta3, ta4, vl1, vl2, vl3, vl4,
                           va1, va2, va3, va4, lrRes1, lrOut1, lrRes2, lrOut2, lrRes3,
                           lrOut3, lrRes4, lrOut4, epochs, path_name):
    fig = PLT.figure()
    PLT.rcParams.update({'font.size': 6})  # Size of text

    ax1 = fig.add_subplot(221)
    ax1.plot(tl1, label="Train_loss_1")
    ax1.plot(tl2, label="Train_loss_2")
    ax1.plot(tl3, label="Train_loss_3")
    ax1.plot(tl4, label="Train_loss_4")
    PLT.legend()
    ax1.set_title("Model 1")
    tl1.extend(tl2)
    tl1.extend(tl3)
    tl1.extend(tl4)
    # maximo1 = max(tl1)
    ax1.set_xlim(0, 10)
    ax1.set_ylim(0, 1.5)

    ax2 = fig.add_subplot(222)
    ax2.plot(ta1, label="Train_acc_1")
    ax2.plot(ta2, label="Train_acc_2")
    ax2.plot(ta3, label="Train_acc_3")
    ax2.plot(ta4, label="Train_acc_4")
    PLT.legend()
    ax2.set_title("Model 2")
    ta1.extend(ta2)
    ta1.extend(ta3)
    ta1.extend(ta4)
    maximo2 = max(ta1)
    ax2.set_xlim(0, epochs+1)
    ax2.set_ylim(0, maximo2)

    ax3 = fig.add_subplot(223)
    ax3.plot(vl1, label="Val_loss_1")
    ax3.plot(vl2, label="Val_loss_2")
    ax3.plot(vl3, label="Val_loss_3")
    ax3.plot(vl4, label="Val_loss_4")
    PLT.legend()
    ax3.set_title("Model 3")
    vl1.extend(vl2)
    vl1.extend(vl3)
    vl1.extend(vl4)
    maximo3 = max(vl1)
    ax3.set_xlim(0, epochs+1)
    ax3.set_ylim(0, maximo3)

    ax4 = fig.add_subplot(224)
    ax4.plot(va1, label="Val_acc_1")
    ax4.plot(va2, label="Val_acc_2")
    ax4.plot(va3, label="Val_acc_3")
    ax4.plot(va4, label="Val_acc_4")
    PLT.legend()
    ax4.set_title("Model 4")
    va1.extend(va2)
    va1.extend(va3)
    va1.extend(va4)
    maximo4 = max(va1)
    ax4.set_xlim(0, epochs+1)
    ax4.set_ylim(0, maximo4)

    PLT.savefig(path_name)
