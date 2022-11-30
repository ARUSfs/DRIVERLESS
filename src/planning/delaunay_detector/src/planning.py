"""
Class to get middle route from estimated circuit


@author: Jacobo Pindado
@date: 20221130
"""
from itertools import permutations

from scipy.spatial import Delaunay
import numpy as np

MAX_DISTANCE = 5

class PlanningSystem():
    """Update tracklimits with new cones detected, calculate
    new path via Delaunay triangulation and smooth it with
    spline.
    """
    ORIGIN = (0.0, 0.0)

    def __init__(self):
        self.cones = None
        self.path = None

    def update_tracklimits(self, cones: list):
        self.path = list(ORIGIN)
        cone_points = list()
        for c in cones:
            if c.confidence > 0.9:
                cone_points.append((c.position.x, c.position.y))

    def calculate_path(self):
        triangles = Delaunay(self.cones)

        for simplex in triangles.simplices:
            for p1, p2 in permutations(simplex, 2):
                if np.linalg.norm(p2 - p1) > MAX_DISTANCE:
                    triangles.add_points
