"""
Script to implement main planning class to get route.

@author: Mariano del Río
@date: 20220320
"""

import numpy as np

from delaunay_detector import delaunay_triangulation
from delaunay_detector import calculate_midpoints
from delaunay_detector import build_path_tree
from delaunay_detector import find_best_path

from utils import spline


class planningSystem():
    """
    Update tracklimits with new cones detected, calculate
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

        for x, y, c, p in cones:
            if p > 0.9:
                if c == 'b':
                    self.tl_left.append(np.array([x, y]))
                elif c == 'y':
                    self.tl_right.append(np.array([x, y]))

    def calculate_path(self):

        path = self.path.copy()

        b_cones = self.tl_left
        y_cones = self.tl_right
        points = np.concatenate(b_cones, y_cones)

        edges = delaunay_triangulation(points, y_cones, b_cones)
        midpoints = calculate_midpoints(edges)
        tree = build_path_tree(path, midpoints)
        path, weight = find_best_path(tree)
        path = spline(path)
        self.path = path

        return self.path
