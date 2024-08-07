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
from scipy.interpolate import splprep, splev
import math
import time


MAX_DISTANCE = rospy.get_param('/delaunay_detector/MAX_DISTANCE')
W_DISTANCE = rospy.get_param('/delaunay_detector/W_DISTANCE')
W_ANGLE = rospy.get_param('/delaunay_detector/W_ANGLE')
W_THRESH = rospy.get_param('/delaunay_detector/W_THRESH')
MAX_ROUTE_LENGTH = rospy.get_param('/delaunay_detector/MAX_ROUTE_LENGTH')
NUM_POINTS = rospy.get_param('/delaunay_detector/NUM_POINTS')
MODE = rospy.get_param('/delaunay_detector/MODE')
SMOOTH = rospy.get_param('/delaunay_detector/SMOOTH')
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

        self.previous_perceptions = []
        self.speed = 0
        self.speed_profile = None
        self.route = None
        self.triang = None
        self.s = None
        self.k = None

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
                            if m[0] not in midpoints_x:
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
                    if m[0] not in midpoints_x:
                        midpoints.append([m[0],m[1]])
                        midpoints_x.append(m[0])
                        midpoints_index.append((i,j))

        if MODE == 'STANLEY':
            opposite_angle = PREV_ANGLE - math.pi if PREV_ANGLE > 0 else PREV_ANGLE + math.pi
            back_path = self.compute_path(np.array(midpoints), np.array([0, 0]), opposite_angle)[1:]
            if len(back_path)>0:
                orig = [back_path[0][0],back_path[0][1]]
                midpoints.remove(orig)
            else: 
                orig = [0,0]
            path = self.compute_path(np.array(midpoints), np.array(orig), PREV_ANGLE)
        else:
            path = self.compute_path(np.array(midpoints), np.array([0, 0]), PREV_ANGLE)
        
        if len(path)>2:
            PREV_ANGLE = np.arctan2(path[1][1], path[1][0])/2

        
        if(SMOOTH):
            # SMOOTHED PATH
            route = np.array(path)
            if(len(route)>2):
                degrees = 3 if len(route)>3 else 2
                tck, u = splprep(route.T, s=3, k=degrees)  
                u_new = np.linspace(u.min(), u.max(), 30)
                route = np.array(splev(u_new, tck)).T

                acum=0
                s=[]
                s.append(0)
                xp = []
                yp = []
                for i in range(route.shape[0]-1):
                    p1=route[i]
                    p2=route[i+1]
                    xp.append(p2[0]-p1[0])
                    yp.append(p2[1]-p1[1])
                    acum+=np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
                    s.append(acum)
                xp.append(xp[-1])
                yp.append(yp[-1])


                xpp=[]
                ypp=[]
                for i in range(len(xp)-1):
                    xpp.append(xp[i+1]-xp[i])
                    ypp.append(yp[i+1]-yp[i])
                xpp.append(xpp[-1])
                ypp.append(xpp[-1])

                k=[]
                for i in range(len(xpp)):
                    if xp[i]!=yp[i]:
                        k.append((xp[i]*ypp[i] - xpp[i]*yp[i])/(xp[i]**2+yp[i]**2)**1.5)
                    else:
                        k.append(0)

                
                speed_profile = [0 for _ in range(len(s))]
                ax_max = 3
                ay_max = 4
                v_max = 8
                v_grip = [min(np.sqrt(ay_max/np.abs(c+0.0001)),v_max) for c in k]
                speed_profile[0] = self.speed
                for j in range(1,len(speed_profile)):
                    ds = s[j]-s[j-1]
                    speed_profile[j] = np.sqrt(speed_profile[j-1]**2 + 2*ax_max*ds)
                    if speed_profile[j] > v_grip[j]:
                        speed_profile[j] = v_grip[j]
                for j in range(len(speed_profile)-2,-1,-1):
                    v_max_braking = np.sqrt(speed_profile[j+1]**2 + 2*ax_max*ds)
                    if speed_profile[j] > v_max_braking:
                        speed_profile[j] = v_max_braking
                
                self.speed_profile = speed_profile 
                self.s = s
                self.k = k
                

        else:
            # UPSAMPLED PATH
            route=[]
            for i in range(len(path)-1):
                route.extend([[(1-a)*path[i][0] + a*path[i+1][0],(1-a)*path[i][1] + a*path[i+1][1]] for a in np.linspace(0,1, num=NUM_POINTS)])
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

        self.route = route
        self.triang = triang

 
    
    def compute_path(self,midpoints, orig, first_angle):
        last_element = orig
        last_angle=first_angle
        acum_distances=0
        non_used_midpoints = np.full(midpoints.shape[0], True, dtype=np.bool_)
        path = [last_element]

        while len(path) < len(midpoints)+1 and acum_distances<MAX_ROUTE_LENGTH:
            vectors = np.full(midpoints.shape, np.Inf)
            distances = np.full(midpoints.shape[0], np.Inf)
            angles = np.full(midpoints.shape[0], np.Inf)
            weights = np.full(midpoints.shape[0], np.Inf)

            vectors[non_used_midpoints] = midpoints[non_used_midpoints] - last_element
            distances[non_used_midpoints] = np.linalg.norm(vectors[non_used_midpoints], axis=1)
            angles[non_used_midpoints] = np.abs(np.arctan2(vectors[non_used_midpoints, 1], vectors[non_used_midpoints, 0])-last_angle)
            angles = np.minimum(angles,np.abs(angles-2*math.pi))

            #esto tiene sentido pero da problemitas, hay que depurar
            # for i in range(len(midpoints)):
            #     max_angle = math.pi/2 if len(path)==1 else math.pi/4
            #     if angles[i]> max_angle:
            #         angles[i]=np.Inf
                        
            weights = W_DISTANCE*distances + W_ANGLE*angles

            next_midpoint = np.nanargmin(weights)
            if weights[next_midpoint] < W_THRESH:
                non_used_midpoints[next_midpoint] = False
                path.append(midpoints[next_midpoint])
                last_element = midpoints[next_midpoint]
                last_angle = np.arctan2(vectors[next_midpoint][1], vectors[next_midpoint][0])
                acum_distances+=distances[next_midpoint]
                # rospy.logwarn([distances[next_midpoint],angles[next_midpoint],weights[next_midpoint]])
            else:
                break
        
        return path

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
