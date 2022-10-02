"""
Classes to split and stitch images in slices that will later be used for YOLO. This allows
for training data and production images to be configured in the right size. This is done
to overcome the size limitations of YOLO detectors.

@author Jacobo Pindado
@date 20211214
"""
import numpy as np
from typing import List, Tuple
from math import ceil

from .BaseClasses import Point, ImageSize, Rectangle, Object


class Slice(Rectangle):
    def __init__(self, tl: Point, slice_size: ImageSize, image: np.ndarray,
                 image_size: ImageSize):

        self.image = image
        self.image_size = image_size
        self.slice_size = slice_size
        self.overlap = slice_size.overlap

        br = tl + Point(slice_size.width, slice_size.height)
        super().__init__(tl, br)

        # Numpy slices have their coordinates flipped
        self.slice_image = image[self.tl.y:self.br.y, self.tl.x:self.br.x, ::]

    def slice_objects(self,
                      objects: List[Object],
                      min_size: int = 0,
                      pad_shift: Point = None):
        rObjects = list()

        for o in objects:
            obj_ = o.get_shifted(pad_shift).intersect(self)

            if obj_ is None \
               or obj_.width < min_size \
               or obj_.height < min_size:
                continue

            obj_ = obj_.get_shifted(self.tl, reverse=True)
            obj_.image_size = self.slice_size

            rObjects.append(obj_)

        return rObjects

    def __add__(self, other):
        if isinstance(other, Point):
            rtl = self.tl + other

            return Slice(rtl, self.slice_size, self.image, self.image_size)

    def __sub__(self, other):
        if isinstance(other, Point):
            rtl = self.tl - other

            return Slice(rtl, self.slice_size, self.image, self.image_size)

    __rsub__ = __sub__

    def __str__(self):
        return "Slice([{0},{1}])".format(self.tl, self.br)


class SliceIterator:
    # If w_m is the maximum width, w is the input image width and o is the overlap, the number
    # of rows will be (w-w_m)//(w_m-o) + 2. This is the same as floor((w-w_m)/(w_m-o)) + 1.
    # Idem. for number of columns.
    def __init__(self, image: np.ndarray, image_size: ImageSize, slice_size: ImageSize,
                 objects: List[Object] = None,
                 min_object_size: int = 0,
                 output_image_dims: Tuple[int] = None):

        self.slice_size = slice_size
        self.objects = objects
        self.min_object_size = min_object_size

        # Calculating the number of total slices, columns and rows
        if output_image_dims is None:
            self.cols, self.rows, output_width, output_height = SliceIterator.output_image_size(image_size, slice_size)  # noqa
        else:
            self.cols, self.rows, output_width, output_height = output_image_dims

        self.total = self.cols * self.rows
        self.current = 0

        # This image will be the one used for the slices. We need this so that every piece has the
        # same size. Padding will be applied equaly so that the resulting image is centered. This is
        # to prevent posible slice configurations causing mostly black images.
        self.pad_width = ceil((output_width - image_size.width)/2)
        self.pad_height = ceil((output_height - image_size.height)/2)
        self.pad_shift = Point(self.pad_width, self.pad_height)

        self.padded_image = np.pad(image,
                                   [(self.pad_height, self.pad_height),
                                    (self.pad_width, self.pad_width), (0, 0)],
                                   mode='constant',
                                   constant_values=0)

        # Elements for iterator
        self.col_shift = Point(slice_size.width, 0)
        self.row_shift = Point(0, slice_size.height)
        self.initial_slice = Slice(Point(0, 0), self.slice_size, self.padded_image, image_size)

    def __iter__(self):
        return self

    def __next__(self):
        if self.current >= self.total:
            raise StopIteration

        cur_col = self.current % self.cols
        cur_row = self.current // self.cols

        shift = self.col_shift*cur_col + self.row_shift*cur_row

        slice = self.initial_slice + shift

        rObjects = slice.slice_objects(self.objects,
                                       min_size=self.min_object_size,
                                       pad_shift=self.pad_shift)

        self.current += 1
        return cur_col, cur_row, slice, rObjects

    def output_image_size(image_size: ImageSize, slice_size: ImageSize):
        cols = ceil((image_size.width-slice_size.width)/(slice_size.width-slice_size.overlap)) + 1
        rows = ceil((image_size.height-slice_size.height)/(slice_size.height-slice_size.overlap))+1

        output_width = slice_size.width + (slice_size.width - slice_size.overlap)*(cols - 1)

        output_height = slice_size.height + (slice_size.height - slice_size.overlap)*(rows - 1)

        return cols, rows, output_width, output_height
