"""
Class to get middle route from estimated circuit


@author: Jacobo Pindado
@date: 20221130
"""
from itertools import combinations

import rospy
import numpy as np
from scipy.spatial import Delaunay
from sklearn.neighbors import NearestNeighbors

from common_msgs.msg import Simplex, Triangulation
from geometry_msgs.msg import Point


MAX_DISTANCE = 8


class PlanningSystem():
    """Update tracklimits with new cones detected, calculate
    new path via Delaunay triangulation and smooth it with
    spline.
    """
    ORIGIN = (0.0, 0.0)

    def __init__(self):
        self.cones = None
        self.colours = None
        self.path = None
        self.distances = None

        self.delaunay_publisher = rospy.Publisher('/delaunay_detector/simplices',
                                                  Triangulation,
                                                  queue_size=1)

    def update_tracklimits(self, cones: list):
        self.colours = list()
        cone_points = list()
        for c in cones:
            if c.confidence > 0.5:
                cone_points.append((c.position.x, c.position.y))
                self.colours.append(c.color)
        self.cones = np.array(cone_points)
        self.distances = dict()

    def calculate_path(self):
        triangles = Delaunay(self.cones)

        preproc_simplices = list()
        midpoints = list()

        for simplex in triangles.simplices:
            if all(self.get_distance(self.cones[p1], self.cones[p2]) < MAX_DISTANCE
                   for p1, p2 in combinations(simplex, 2)):
                for p1, p2 in combinations(simplex, 2):  # TODO Could be optimized.
                    print(self.colours[p1], self.colours[p2])
                    if not self.colours[p1] == self.colours[p2]:
                        midpoints.append((self.cones[p1] + self.cones[p2])/2)
                        preproc_simplices.append(simplex)
                        break

        self.publish_delaunay_triangles(preproc_simplices)
        midpoints = np.array(midpoints)
        neighbors = NearestNeighbors(n_neighbors=7, algorithm='auto').fit(midpoints)

        last_element = np.array((0, 0))
        path_indices = list()
        stop = False
        while len(path_indices) < len(midpoints) and not stop:
            indices = neighbors.kneighbors([last_element], return_distance=False)
            stop = True
            for i in indices[0]:
                if i not in path_indices:
                    path_indices.append(i)
                    last_element = midpoints[i]
                    stop = False
                    break

        route = np.empty((len(path_indices) + 1, 2))
        route[0, :] = np.zeros((1, 2))
        route[1:, :] = midpoints[path_indices]

        # TODO: I don't think using a spline to "smooth" the route is worth since
        # pure_pursuit will do it anyway with parameter changes. As of now it will
        # remain un-splined.
        return route

    def get_distance(self, p1, p2):
        p1b = p1.tobytes()
        p2b = p2.tobytes()
        if (p1b, p2b) in self.distances:
            return self.distances.get((p1b, p2b))
        elif (p2b, p1b) in self.distances:
            return self.distances.get((p2b, p1b))
        else:
            distance = np.linalg.norm(p1 - p2)
            self.distances[(p1b, p2b)] = distance
            return distance

    def publish_delaunay_triangles(self, simplices):
        triang = Triangulation()
        triang.simplices = list()
        for simplex_indices in simplices:
            simplex = Simplex()
            simplex.simplex = list()
            for i in simplex_indices:
                point = Point()
                point.x = self.cones[i, 0]
                point.y = self.cones[i, 1]
                simplex.simplex.append(point)

            triang.simplices.append(simplex)
        self.delaunay_publisher.publish(triang)
