"""
Class to get middle route from estimated circuit


@author: Jacobo Pindado
@date: 20221130
"""
from itertools import combinations, permutations

import rospy
import numpy as np
from scipy.spatial import Delaunay
from scipy.interpolate import BSpline

from common_msgs.msg import Simplex, Triangulation
from geometry_msgs.msg import Point


MAX_DISTANCE = 8
W_DISTANCE = 1
W_ANGLE = 4
W_THRESH = 8
PREV_ANGLE = 0


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

    def update_tracklimits(self, cones):
        self.colours = list()
        cone_points = list()
        for c in cones:
            if c[4] > 0.5 and c[3] != 2:
                cone_points.append((c[0], c[1]))
                self.colours.append(c[3])
        self.cones = np.array(cone_points)
        self.distances = dict()

    def calculate_path(self):

        global PREV_ANGLE

        if len(self.cones) < 3:
            return list(), list()
        triangles = Delaunay(self.cones)

        preproc_simplices = list()
        midpoints = list()
        midpoints_x = list()
        midpoints_index = list()

        if(rospy.get_param('~color_enabled')):

            for simplex in triangles.simplices:
                if all(self.get_distance(self.cones[p1], self.cones[p2]) < MAX_DISTANCE
                        for p1, p2 in combinations(simplex, 2)):
                    for p1, p2 in combinations(simplex, 2):  # TODO Could be optimized.
                        if not self.colours[p1] == self.colours[p2]:
                            m = (self.cones[p1] + self.cones[p2])/2
                            if m[0] not in midpoints_x and m[0]>0:
                                midpoints.append(m)
                                midpoints_x.append(m[0])
                            preproc_simplices.append(simplex)
                            # si quito el break hay el doble de midpoints
                            # break

        
        else:
            valid_simplices = [True for _ in range(len(triangles.simplices))]
            for i in range(len(triangles.simplices)):
                simplex = triangles.simplices[i]
                for a,b,c in permutations(simplex, 3):
                    vectA = self.cones[b] - self.cones[a]
                    vectB = self.cones[c] - self.cones[a]
                    
                    angle = np.degrees(np.arccos(np.dot(vectA, vectB) / (np.linalg.norm(vectA) * np.linalg.norm(vectB))))

                    if angle>=130 or angle<=15:
                        valid_simplices[i]=False
                        break

            dict = {}
            for i in range(len(triangles.simplices)):
                simplex = triangles.simplices[i]
                if all(self.get_distance(self.cones[i], self.cones[j]) < MAX_DISTANCE
                        for i,j in combinations(simplex, 2)) and valid_simplices[i]:
                    for (i,j) in combinations(simplex,2):
                        if (i,j) in dict.keys():
                            dict[(i,j)]+=1
                        elif (j,i) in dict.keys():
                            dict[(j,i)]+=1
                        else:
                            dict[(i,j)]=1

                    preproc_simplices.append(simplex)

            
            for (i,j) in dict.keys():
                if dict[(i,j)]>=2:
                    m = (self.cones[i] + self.cones[j])/2
                    if m[0] not in midpoints_x and m[0]>0:
                        midpoints.append(m)
                        midpoints_x.append(m[0])
                        midpoints_index.append((i,j))



        last_element = np.array([0, 0])
        last_angle=PREV_ANGLE
        midpoints = np.array(midpoints)
        non_used_midpoints = np.full(midpoints.shape[0], True, dtype=np.bool_)
        path = [last_element]

        while len(path) < len(midpoints)+1:
            vectors = np.full(midpoints.shape, np.Inf)
            distances = np.full(midpoints.shape[0], np.Inf)
            angles = np.full(midpoints.shape[0], np.Inf)
            weights = np.full(midpoints.shape[0], np.Inf)

            vectors[non_used_midpoints] = midpoints[non_used_midpoints] - last_element
            distances[non_used_midpoints] = np.linalg.norm(vectors[non_used_midpoints], axis=1)
            angles[non_used_midpoints] = np.abs(np.arctan2(vectors[non_used_midpoints, 1], vectors[non_used_midpoints, 0])-last_angle)


            weights = W_DISTANCE*distances + W_ANGLE*angles

            next_midpoint = np.nanargmin(weights)
            if weights[next_midpoint] < W_THRESH:
                non_used_midpoints[next_midpoint] = False
                path.append(midpoints[next_midpoint])
                last_element = midpoints[next_midpoint]
                last_angle = np.arctan2(vectors[next_midpoint][1], vectors[next_midpoint][0])
                # rospy.logwarn([distances[next_midpoint],angles[next_midpoint],weights[next_midpoint]])
            else:
                break
        
        if len(path)>1:
            PREV_ANGLE = np.arctan2(path[1][1], path[1][0])/2
        else:
            PREV_ANGLE = 0
        # route = np.array(path)

             
        route=[]
        for i in range(len(path)-1):
            route.extend([[(1-a)*path[i][0] + a*path[i+1][0],(1-a)*path[i][1] + a*path[i+1][1]] for a in np.linspace(0,1, num=5)])
        route = np.array(route)

        triang = Triangulation()
        for simplex in preproc_simplices:
            s = Simplex()
            for ind in simplex:
                p = Point()
                p.x = self.cones[ind][0]
                p.y = self.cones[ind][1]
                p.z = 0
                s.simplex.append(p)
            triang.simplices.append(s)  

        return route, triang

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
