#!/usr/bin/env python3
"""
Script and class to convert from MIT dataset to YOLO format.

@author: Jacobo Pindado
@date: 20211214
"""
import os
import sys
import argparse
import pathlib
from random import shuffle, random

import csv
import cv2
from tqdm import tqdm

sys.path.append(pathlib.PosixPath(os.path.abspath(__file__))
                .parents[2].as_posix())
from detection.utils.ImageSlicer import ImageSize, SliceIterator  # noqa
from detection.utils.BaseClasses import Object  # noqa


INFO_COLUMNS = 5


def main():
    parser = argparser_setup()
    args = parser.parse_args()

    csvfile = args.csv
    images_folder = args.images
    train_file = args.darknet.joinpath('data/train.txt')
    validate_file = args.darknet.joinpath('data/test.txt')
    output_folder = args.darknet.joinpath('data/obj/')
    threshold = args.thresh
    evalPercent = args.evalPercent
    if args.maxSize is not None:
        maxSize = ImageSize(*tuple(args.maxSize))
    else:
        maxSize = None
    emptySliceProb = args.emptyProbability
    display = args.display

    extract(csvfile, images_folder, train_file, validate_file,
            output_folder, threshold, evalPercent, maxSize,
            emptySliceProb, display)
    print("Done!")


# This class receives a line from the MIT csv file
# and parses it to be iterable.
class ImageIterator:
    def __init__(self, csv_file):
        count_reader = open(csv_file, 'r')
        self.total = sum(1 for _ in csv.reader(count_reader))
        count_reader.close()
        self.stream = open(csv_file, 'r')
        self.reader = csv.reader(self.stream, quotechar='"')
        # We ignore the headers
        next(self.reader)

    def __iter__(self):
        return self

    def __next__(self):
        line = next(self.reader, None)
        if line is None:
            raise StopIteration
        image_filename = line[0].replace('.jpg', '')
        image_width = int(line[2])
        image_height = int(line[3])

        # Though unlikely, all the columns may be full. In that case
        # indexof will return ValueError.
        try:
            first_empty = line.index('')
        except ValueError:
            first_empty = len(line)

        # If the image doesn't have any objects, we remove it from our dataset.
        # YOLO doesn't require negative images.
        if first_empty == INFO_COLUMNS:
            return next(self)

        # We parse the strings to list of objects.
        imSize = ImageSize(image_width, image_height)
        objects = [Object.fromMITStr(s, imSize) for s in line[INFO_COLUMNS:first_empty]]

        return image_filename, image_width, image_height, objects

    # We want to close the reader stream once the iterator is no longer used.
    def __del__(self):
        self.stream.close()


def save_image_and_objects(array, objects, filename, folder):
    # Saving the array to file.
    image_dest = folder.joinpath(filename + '.jpg')
    cv2.imwrite(str(image_dest), array)

    # Saving the objets to file.
    objects_dest = folder.joinpath(filename + '.txt')
    with open(objects_dest, 'w+') as output:
        for o in objects:
            output.write(o.to_YOLO_str() + '\n')


# Main function to extract images and objects from original MIT dataset format.
def extract(csvfile, images_folder, train_file, validate_file,
            output_folder, thresh: int = 5, evalPercent: int = 10,
            maxSize: ImageSize = None, emptySliceProb: int = 1,
            display: bool = False) -> None:
    if not thresh >= 0:
        raise ValueError("thresh must be a non negative integer")
    elif not (evalPercent >= 0 and evalPercent <= 100):
        raise ValueError("evalPercent must be an integer between 0 and 100")

    names = []

    iterator = ImageIterator(csvfile)
    pbar = tqdm(total=iterator.total)
    for image_filename, width, height, objects in ImageIterator(csvfile):
        pbar.update(1)
        # We load the original image as a NumpyArray.
        image_location = images_folder.joinpath(image_filename + '.jpg')
        im = cv2.imread(str(image_location))

        if maxSize is None:
            save_image_and_objects(im, objects, image_filename, output_folder)
            names.append(image_filename + '.jpg')
            if display:
                for obj in objects:
                    cv2.rectangle(im, obj.tl.tuple(), obj.br.tuple(),
                                  (0, 0, 255), thickness=1)
                cv2.imshow("Images", im)
                cv2.waitKey(0)

        else:

            originalSize = ImageSize(width, height)

            for cur_col, cur_row, slice, slice_objects in SliceIterator(im, originalSize,
                                                                        maxSize, objects, thresh):
                slice_filename = '{0}_{1:02d}_{2:02d}'.format(image_filename, cur_col, cur_row)
                # Adding the check for != 1 to make it faster in that case.
                if len(slice_objects) == 0 and emptySliceProb != 1 and random() > emptySliceProb:
                    continue

                save_image_and_objects(slice.slice_image,
                                       slice_objects,
                                       slice_filename,
                                       output_folder)
                names.append(slice_filename + '.jpg')

                if display:
                    for obj in slice_objects:
                        cv2.rectangle(slice.slice_image, obj.tl.tuple(),
                                      obj.br.tuple(), (0, 0, 255), thickness=1)
                    cv2.imshow("Slices", slice.slice_image)
                    cv2.waitKey(0)

    if display:
        cv2.destroyAllWindows()

    pbar.close()

    with open(train_file, 'w+') as train, open(validate_file, 'w+') as validate:
        # To override file
        train.seek(0)
        validate.seek(0)
        shuffle(names)
        separation_point = len(names)*evalPercent/100
        for i, name in enumerate(names):
            if i < separation_point:
                validate.write("data/obj/" + name + '\n')
            else:
                train.write("data/obj/" + name + '\n')


def argparser_setup():
    parser = argparse.ArgumentParser(description='Convert MIT cone DB to YOLO format.')
    parser.add_argument('csv', metavar='csv',
                        help='csv file where names and positions are stored')
    parser.add_argument('images', metavar='imgs', type=pathlib.Path,
                        help='folder where original images are stored')
    parser.add_argument('darknet', metavar='darknet', type=pathlib.Path,
                        help='darknet path to store results')
    parser.add_argument('--evalPercent', metavar='e%', type=int, default=10, nargs='?',
                        help='percentage of images for evaluation of net')
    parser.add_argument('--thresh', metavar='thr', type=int, default=0, nargs='?',
                        help='minimum cone size in px')
    parser.add_argument('--maxSize', metavar='px', type=int, nargs=3,
                        help='size (width height overlap) of the slices to be extracted \
                        from each image')
    parser.add_argument('--emptyProbability', metavar='ep', type=float, default=1, nargs='?',
                        help='Probability that a slice with no objects will be saved.')
    parser.add_argument('--display', metavar='dsp', action=argparse.BooleanOptionalAction,
                        help='Display images as they\'re being sliced.')
    parser.set_defaults(display=False)
    return parser


if __name__ == "__main__":
    main()
