"""
This file includes various classes that will be usefull for many parts of cone detection. Mainly
ImageSize to store width, heights and overlaps; and Object to store detections in
it's different formats.

@author Jacobo Pindado
@author 20211214
"""
from dataclasses import dataclass
from math import floor
from copy import deepcopy

# We use this dataclass for readability and to ease type checking.
@dataclass
class ImageSize:
    width: int
    height: int
    overlap: int = 0

    def __post_init__(self):
        for k, v in vars(self).items():
            if v is None:
                raise ValueError("The parameter {0} can't be of NoneType".format(k))


@dataclass
class Point:
    x: float
    y: float

    def __add__(self, other):
        if isinstance(other, Point):
            rx = self.x + other.x
            ry = self.y + other.y
            return Point(rx, ry)
        else:
            raise TypeError("Cannot add {0} and {1}"
                            .format(type(self).__name__, type(other).__name__))

    def __sub__(self, other):
        if isinstance(other, Point):
            rx = self.x - other.x
            ry = self.y - other.y
            return Point(rx, ry)
        else:
            raise TypeError("Cannot subtract {0} and {1}"
                            .format(type(self).__name__, type(other).__name__))

    def __mul__(self, other):
        if isinstance(other, Point):
            rx = self.x*other.x
            ry = self.y*other.y
            return Point(rx, ry)
        if isinstance(other, int) or isinstance(other, float):
            rx = self.x*other
            ry = self.y*other
            return Point(rx, ry)
        else:
            raise TypeError("Cannot multiply {0} and {1}"
                            .format(type(self).__name__, type(other).__name__))

    __rmul__ = __mul__

    def __truediv__(self, other):
        if isinstance(other, Point):
            rx = self.x/other.x
            ry = self.x/other.y
            return Point(rx, ry)
        elif isinstance(other, float) or isinstance(other, int):
            rx = self.x/other
            ry = self.y/other
            return Point(rx, ry)
        else:
            raise TypeError("Cannot divide {0} and {1}"
                            .format(type(self).__name__, type(other).__name__))

    def to_int(self):
        self.x = floor(self.x)
        self.y = floor(self.y)

    def tuple(self):
        return self.x, self.y

    def __str__(self):
        return "({0},{1})".format(self.x, self.y)


class Rectangle:

    def __init__(self, tl: Point, br: Point):
        if tl.x > br.x or tl.y > br.y:
            raise ValueError("tl must be to the left and over br")

        self.tl = tl
        self.br = br
        self.width = br.x - tl.x
        self.height = br.y - tl.y

    def copy(self):
        return deepcopy(self)

    def get_shifted(self, point: Point, reverse: bool = False):
        obj_ = self.copy()
        xShift = point.x*(-1 if reverse else 1)
        yShift = point.y*(-1 if reverse else 1)
        obj_.tl.x += xShift
        obj_.tl.y += yShift
        obj_.br.x += xShift
        obj_.br.y += yShift
        return obj_

    def overlapping_rectangle(self, rect):
        rx1 = min(self.tl.x, rect.tl.x)
        ry1 = min(self.tl.y, rect.tl.y)
        rx2 = max(self.br.x, rect.br.x)
        ry2 = max(self.br.y, rect.br.y)

        rtl = Point(rx1, ry1)
        rbr = Point(rx2, ry2)
        return Rectangle(rtl, rbr)

    def intersect(self, other):
        if isinstance(other, Rectangle):
            rx1 = max(self.tl.x, other.tl.x)
            ry1 = max(self.tl.y, other.tl.y)
            rx2 = min(self.br.x, other.br.x)
            ry2 = min(self.br.y, other.br.y)

            rtl = Point(rx1, ry1)
            rbr = Point(rx2, ry2)
            # This would be an empty intersection.
            if rtl.x >= rbr.x or rtl.y >= rbr.y:
                return None
            return Rectangle(rtl, rbr)
        elif isinstance(other, list):
            rRect = self
            for r in other:
                rRect.intersect(r)

            return rRect
        else:
            raise TypeError("Cannot intersect {0} and {1}"
                            .format(type(self).__name__, type(other).__name__))

    def area(self):
        return self.width * self.height

    def union_area(self, other):
        if isinstance(other, Rectangle):
            intersection = self.intersect(other)
            inter_area = 0 if intersection is None else intersection.area()
            return self.area() + other.area() - inter_area
        else:
            raise TypeError("Cannot intersect {0} and {1}"
                            .format(type(self).__name__, type(other).__name__))

    def to_int(self):
        self.tl.to_int()
        self.br.to_int()

        self.width = floor(self.width)
        self.height = floor(self.height)

    def __str__(self):
        return "{0},{1}".format(self.tl.__str__, self.br.__str__)


class Object(Rectangle):
    # TODO include object name and/or id for future
    # object_class = 0

    def __init__(self, tl: Point, width: int, height: int,
                 object_class: int, image_size: ImageSize):
        self.image_size = image_size
        self.object_class = object_class

        br = tl + Point(width, height)
        super().__init__(tl, br)

    def __str__(self):
        return "Obj({0}, [{1},{2}])".format(self.object_class, self.tl, self.br)

    __repr__ = __str__

    def __add__(self, other):
        if isinstance(other, Object) or isinstance(other, Rectangle):
            rx1 = min(self.tl.x, other.tl.x)
            ry1 = min(self.tl.y, other.tl.y)
            rx2 = max(self.br.x, other.br.x)
            ry2 = max(self.br.y, other.br.y)

            rtl = Point(rx1, ry1)
            rbr = Point(rx2, ry2)
            return Object.from_points(rtl, rbr, self.object_class, self.image_size)
        elif isinstance(other, Point):
            return other + self
        else:
            raise TypeError("Cannot add {0} and {1}"
                            .format(type(self).__name__, type(other).__name__))

    def from_rect(rect: Rectangle, object_class: int, image_size: ImageSize):
        return Object(rect.tl, rect.width, rect.height, object_class, image_size)

    def from_points(tl: Point, br: Point, object_class: int, image_size: ImageSize):
        width = br.x - tl.x
        height = br.y - tl.y
        return Object(tl, width, height, object_class, image_size)

    def intersect(self, other):
        if isinstance(other, Rectangle):
            rRect = Rectangle.intersect(self, other)
            if rRect is not None:
                return Object(rRect.tl, rRect.width, rRect.height,
                              self.object_class, self.image_size)
            else:
                return None
        elif isinstance(other, Object):
            if self.object_class == other.object_class:
                rRect = super().intersect(other)
                if rRect is not None:
                    return Object(rRect.tl, rRect.width, rRect.height,
                                  self.object_class, self.image_size)
                else:
                    return None
            else:
                return None

    def fromMIT(x1: int, y1: int, height: int, width: int,
                image_size: ImageSize, object_class: int = 0):
        if x1 is None or y1 is None:
            raise ValueError("The coordinates of the point can't be of NoneType")
        elif height is None or width is None:
            raise ValueError("Width and height values can't be of NoneType")
        elif image_size is None:
            raise ValueError("image_size can't be of NoneType")

        rtl = Point(x1, y1)

        return Object(rtl, width, height, object_class, image_size)

    def fromMITStr(string: str, image_size: ImageSize):
        def str2tuple(s): return tuple(int(n) for n in s.strip("[]").split(","))
        return Object.fromMIT(*str2tuple(string), image_size)

    def to_YOLO(self):
        center = (self.tl + self.br)/2
        center.x /= self.image_size.width
        center.y /= self.image_size.height

        rwidth = self.width / self.image_size.width
        rheight = self.height / self.image_size.height

        assert 0 < center.x < 1 \
            and 0 < center.y < 1 \
            and 0 < rwidth < 1 \
            and 0 < rheight < 1

        return center, rwidth, rheight

    def from_YOLO(center: Point, width: float, height: float, object_class: int, image_size):
        # factor = Point(image_size.width, image_size.height)
        wh_vector = Point(width, height)/2

        rtl = (center - wh_vector)  # * factor
        # rwidth *= image_size.width
        # rheight *= image_size.height

        return Object(rtl, width, height, object_class, image_size)

    def to_YOLO_str(self):
        center, width, height = self.to_YOLO()
        output_string = "{0} {1:06f} {2:06f} {3:06f} {4:06f}".format(0,
                                                                     center.x,
                                                                     center.y,
                                                                     width,
                                                                     height)
        return output_string
