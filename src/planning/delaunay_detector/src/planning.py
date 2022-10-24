"""Script to implement main planning class to get route.

@author: Mariano del Río
@date: 20220320
"""

import numpy as np

from delaunay_detector import delaunay_triangulation
from delaunay_detector import build_path_tree
from delaunay_detector import find_best_path

from utils import spline


class PlanningSystem():
    """Update tracklimits with new cones detected, calculate
    new path via Delaunay triangulation and smooth it with
    spline.
    """

    def __init__(self):

        self.origin = np.zeros(2)
        self.tl_left = []
        self.tl_right = []

        self.path = []
        self.path.append(self.origin)

    def update_tracklimits(self, cones: list):

        self.tl_left = []
        self.tl_right = []

        for c in cones:
            if c.probability.data > 0.9:
                if c.color.data == 'b':
                    self.tl_left.append(np.array([c.position.x, c.position.y]))
                elif c.color.data == 'y':
                    self.tl_right.append(np.array([c.position.x, c.position.y]))

    def calculate_path(self):

        path = self.path.copy()

        points = np.array(self.tl_left + self.tl_right)

        midpoints = delaunay_triangulation(points, self.tl_left, self.tl_right)
        tree = build_path_tree(path, midpoints)
        path, weight = find_best_path(tree)
        path = spline(path)
        self.path = path

        return self.path
