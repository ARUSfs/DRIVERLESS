#!/usr/bin/env python3
"""
Classes and script to split and stitch images in slices to be separately processed
by darknet.py, a python wrapper for darknet.

@author: Jacobo Pindado
@date: 20211214
"""
import sys
import argparse
import configparser
from pathlib import Path
import cv2
import numpy as np

from typing import List, Tuple
from .utils.BaseClasses import ImageSize, Point, Rectangle, Object
from .utils.ImageSlicer import Slice, SliceIterator
from .utils import darknet


class Detector:

    def __init__(self, cfg: configparser.ConfigParser,
                 input_size: ImageSize = None,
                 slice_size: ImageSize = None):
        self.cfg = cfg['YOLO']['config']
        self.data = cfg['YOLO']['data']
        self.weights = cfg['YOLO']['weights']

        # TODO number - name translation
        self.network, self.names, self.colors = darknet.load_network(self.cfg, self.data,
                                                                     self.weights)

        # TODO this shouldn't be hardcoded
        self.slice_size = ImageSize(480, 360, 0)

        self.input_size = input_size
        if input_size is not None:
            self.output_size = SliceIterator.output_image_size(self.input_size, self.slice_size)
        else:
            self.output_size = None

    def detect_image(self, image: np.ndarray, UoO_threshold: int = 0.6):

        if self.input_size is None:
            self.input_size = ImageSize(image.shape[1], image.shape[0])
        if self.output_size is None:
            self.output_size = SliceIterator.output_image_size(self.input_size, self.slice_size)

        object_dict = dict()
        final_objects = list()

        iterator = SliceIterator(image, self.input_size, self.slice_size,
                                 output_image_dims=self.output_size)
        for col, row, slice, _ in iterator:
            predictions = self.detect_from_slice(slice)

            object_dict[(col, row)] = (slice, predictions)
            if col > 0:
                Detector.join_objects((col - 1, row), predictions, object_dict, UoO_threshold)
            if row > 0:
                Detector.join_objects((col, row - 1), predictions, object_dict, UoO_threshold)
            if col > 0 and row > 0:
                Detector.join_objects((col - 1, row - 1), predictions, object_dict, UoO_threshold)

        for _, object_list in object_dict.values():
            if object_list is not None:
                final_objects.extend(o.get_shifted(Point(iterator.pad_width, iterator.pad_height),
                                                   reverse=True) for o in object_list)

        return final_objects

    def detect_video(video_path: str, output_path: str,
                     detector_cfg: configparser.ConfigParser,
                     UoO_threshold: int = 0.6):
        # from tqdm import tqdm
        cap = cv2.VideoCapture(video_path)

        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        # frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)

        input_size = ImageSize(frame_width, frame_height)
        detector = Detector(detector_cfg, input_size=input_size)

        fourcc = cv2.VideoWriter_fourcc(*'H264')
        out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))
        # progress_bar = tqdm(total=frames)
        while(cap.isOpened()):
            # progress_bar.update(1)
            ret, frame = cap.read()

            if ret:
                objects = detector.detect_image(frame)
                Detector.draw_objects(frame, objects)

                out.write(frame)
            else:
                break

        out.release()
        cap.release()

    def union_over_overlap(rect1: Rectangle, rect2: Rectangle):
        return rect1.union_area(rect2) / rect1.overlapping_rectangle(rect2).area()

    def detect_from_slice(self, slice: Slice):
        image = darknet.nparray_to_image(slice.slice_image)

        predictions = darknet.detect_image(self.network, self.names, image)
        darknet.free_image(image)
        return Detector.predictions_to_object(predictions, slice)

    def join_objects(other_position: Tuple[int, int],
                     predictions: List[Object],
                     object_dict: dict,
                     UoO_threshold: float):
        _, objects = object_dict[other_position]
        for p in predictions:
            candidate_objects = list()
            for o in objects:
                UoO = Detector.union_over_overlap(o, p)

                if UoO >= UoO_threshold:
                    candidate_objects.append((o, UoO))

            if len(candidate_objects) != 0:
                obj = max(candidate_objects, key=lambda k: k[1])[0]

                objects.remove(obj)
                predictions.remove(p)
                objects.append(obj.overlapping_rectangle(p))

    def predictions_to_object(predictions: List[Object], slice: Slice):
        rObjects = list()
        for pred in predictions:
            center = Point(pred[2][0], pred[2][1])
            width = pred[2][2]
            height = pred[2][3]

            object = Object.from_YOLO(center, width, height, '0', slice.image_size)
            object.to_int()

            if object.area() > 0:
                rObjects.append(object.get_shifted(slice.tl))

        return rObjects

    def draw_objects(image: np.ndarray, objects: List[Object]):
        for o in objects:
            cv2.rectangle(image, o.tl.tuple(), o.br.tuple(), (0, 0, 255), thickness=2)


def argparser_setup():
    parser = argparse.ArgumentParser(description='Detect cones from given\
                                     image with trained weights')
    subparsers = parser.add_subparsers(help='command')

    image_parser = subparsers.add_parser('image', help='Detect on images')
    image_parser.set_defaults(func=image_command)
    image_parser.add_argument("configfile", metavar='<cfg>',
                              help="Path to config file", type=Path)
    image_parser.add_argument("imagepath", metavar='<img path>',
                              help="Path to image to detect", type=Path)
    image_parser.add_argument("--outfile", metavar='<out path>',
                              help="Path where resulting image will be outputed. If not specified,\
                              cone positions will be printed",
                              type=Path)

    video_parser = subparsers.add_parser('video', help='Detect on videos')
    video_parser.set_defaults(func=video_command)
    video_parser.add_argument('configfile', metavar='<cfg>',
                              help="Path to config file", type=Path)
    video_parser.add_argument("videopath", metavar='<vid path>',
                              help="Path to video to detect", type=Path)
    video_parser.add_argument("outfile", metavar='<out path>',
                              help="Path where resulting video will be outputed",
                              type=Path)
    return parser


def image_command(args: argparse.ArgumentParser):
    config = configparser.ConfigParser()
    config.read(str(args.configfile.resolve()))
    input_file = str(args.imagepath.resolve())
    output_file = str(args.outfile.resolve()) if args.outfile is not None else None
    detector = Detector(config)

    image = cv2.imread(input_file)
    objects = detector.detect_image(image)
    if output_file is None:
        print(objects)
    else:
        Detector.draw_objects(image, objects)
        cv2.imwrite(output_file, image)


def video_command(args):
    config = configparser.ConfigParser()
    config.read(str(args.configfile.resolve()))
    input_file = str(args.videopath.resolve())
    output_file = str(args.outfile.resolve())

    Detector.detect_video(input_file, output_file, config)


def configparser_read(file: str):
    return configparser.ConfigParser().read(file)


if __name__ == "__main__":
    args = argparser_setup().parse_args()
    args.func(args)
    sys.exit()
