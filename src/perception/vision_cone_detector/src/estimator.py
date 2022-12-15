#!/usr/bin/env python3
"""
Class that will be used by rospy to pass each image through all the
perception pipeline. The estimator will take an image as an input,
and will output an estimation of the position of each cone relative
to the car.

@author: Jacobo Pindado
@date: 20221030
"""
import os
from typing import Union, List, Tuple
from dataclasses import dataclass
import time

import cv2
import numpy as np
import rospy

from detector.utils import darknet
from detector import YoloDetector
from keypoints.keypoint_detector import keypoint_detector


@dataclass
class CameraConfig:
    camera_mat: np.ndarray
    dist_coefs: np.ndarray
    homography_mat: np.ndarray
    new_camera_mat: np.ndarray
    inv_new_camera_mat: np.ndarray
    transform_matrix: np.ndarray
    img_format: str

    def from_param(cam_info):
        camera_mat = np.array(cam_info["camera_mat"]).reshape((3, 3))
        dist_coefs = np.array(cam_info["dist_coefs"])
        homography_mat = np.array(cam_info["homography_mat"])\
                           .reshape((3, 3))

        new_camera_mat = cv2.getOptimalNewCameraMatrix(camera_mat, dist_coefs, (1920, 1088), 0)[0]

        inv_new_camera_mat = np.linalg.inv(new_camera_mat)

        transform_matrix = homography_mat @ inv_new_camera_mat

        conf = CameraConfig(camera_mat=camera_mat,
                            dist_coefs=dist_coefs,
                            homography_mat=homography_mat,
                            new_camera_mat=new_camera_mat,
                            inv_new_camera_mat=inv_new_camera_mat,
                            transform_matrix=transform_matrix,
                            img_format=cam_info["img_format"])
        return conf


class Estimator:
    def __init__(self,
                 yolo_cfg: Union[str, bytes, os.PathLike],
                 yolo_data: Union[str, bytes, os.PathLike],
                 yolo_weights: Union[str, bytes, os.PathLike],
                 kpt_num: int,
                 kpt_weights: Union[str, bytes, os.PathLike],
                 kpt_img_size: Tuple[int, int],
                 camera: CameraConfig):

        self.yolo_detector = YoloDetector(yolo_cfg, yolo_data, yolo_weights)
        self.kpt_detector = keypoint_detector(kpt_num, kpt_weights, kpt_img_size)
        self.cam_conf = camera

    def map_from_image(self,
                       img: np.ndarray) -> List[Tuple[str, float, Tuple[float, float]]]:

        if img.shape != (1088, 1920, 3):
            img = cv2.resize(img, (1920, 1088))
        img = cv2.undistort(img, self.cam_conf.camera_mat,
                            self.cam_conf.dist_coefs, None,
                            self.cam_conf.new_camera_mat)

        yolo_time = time.time()
        detections = self.yolo_detector.predict_from_image(img)
        yolo_time = time.time() - yolo_time

        vertex_array = np.empty((3, len(detections)))
        vertex_array[2, :] = np.ones(len(detections))
        vertex_properties = list()

        map_list = list()

        kpt_time = 0
        np_time = 0
        for i, (class_name, confidence, bbox) in enumerate(detections):
            xmin, ymin, xmax, ymax = darknet.bbox2points(bbox)
            # Due to rounding errors, boxes may fall outside the image
            t = time.time()
            xmin = max(0, xmin)
            ymin = max(0, ymin)
            xmax = min(xmax, img.shape[1] - 1)
            ymax = min(ymax, img.shape[0] - 1)
            cone_bbox_img = img[ymin:ymax, xmin:xmax]

            # keypoints relative to bounding box
            kpts_relative = self.kpt_detector.get_keypoint_from_image(cone_bbox_img)
            kpt_time = kpt_time + ttime.time() - t + 1

            # Right now, we only apply homography to the vertex of the cone. Detecting 7 kpts
            # is unnecesary, but is left for future use. We also transform from relative to
            # absolute to whole image.
            vertex_array[:2, i] = kpts_relative[0] + np.array([xmin, ymin])



            vertex_properties.append((class_name, float(confidence)/100))

        t = time.time()
        vertex_array = self.cam_conf.transform_matrix @ vertex_array
        vertex_array /= vertex_array[2, :]  # We normalize the affine point
        np_time = time.time() - t

        rospy.logwarn(f'yolo: {yolo_time} kpt: {kpt_time} np: {np_time}')
        return list((class_name, conf, (p[0], p[1])) for (class_name, conf), p\
                     in zip(vertex_properties, vertex_array))
