#!/usr/bin/env python3
"""
Classes and script to split and stitch images in slices to be separately processed
by darknet.py, a python wrapper for darknet.

@author: Jacobo Pindado
@date: 20211214
"""
import os
from typing import List, Union

import cv2
import numpy as np

from .utils import darknet


class YoloDetector:
    def __init__(self,
                 cfg: Union[str, bytes, os.PathLike],
                 data: Union[str, bytes, os.PathLike],
                 weights: Union[str, bytes, os.PathLike]):

        self.network, self.names, self.colors = darknet.load_network(cfg, data, weights)
        self.darknet_width = darknet.network_width(self.network)
        self.darknet_height = darknet.network_height(self.network)

    def predict_from_image(self, img: np.ndarray) -> List[darknet.DETECTION]:
        """Method to infere image through a given darknet network.

        :param img: Numpy array with image data. Channel order **must** be RGB.
                    When using cv2 for capturing, this should be taken into
                    account since it works with BRG.
        :type img:  np.ndarray
        :return: List containing Darknet detections
        :rtype: List[darknet.DETECTION]
        """
        frame = cv2.resize(img,
                           (self.darknet_width, self.darknet_height),
                           interpolation=cv2.INTER_LINEAR)
        darknet_image = darknet.make_image(self.darknet_width, self.darknet_height, 3)
        darknet.copy_image_from_bytes(darknet_image, frame.tobytes())

        detections = darknet.detect_image(self.network, self.names, darknet_image)
        darknet.free_image(darknet_image)

        return detections
