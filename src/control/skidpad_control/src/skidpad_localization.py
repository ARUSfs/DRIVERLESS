#!/usr/bin/env python3

import random
import rospy
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class SkidpadLocalization():

    def __init__(self):
        self.N_iterations = 500
        self.threshold = 0.3

    def find_circle_center(self, point1, point2, point3):
        # Paso 1: Calcular los puntos medios
        mid_point1 = ((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2)
        mid_point2 = ((point2[0] + point3[0]) / 2, (point2[1] + point3[1]) / 2)

        # Paso 2: Calcular las pendientes de las mediatrices
        slope1 = -(point2[0] - point1[0]) / (point2[1] - point1[1]) if point2[1] != point1[1] else float('inf')
        slope2 = -(point3[0] - point2[0]) / (point3[1] - point2[1]) if point3[1] != point2[1] else float('inf')

        # Paso 3: Calcular las ecuaciones de las mediatrices
        if slope1 != float('inf'):
            intercept1 = mid_point1[1] - slope1 * mid_point1[0]
        else:
            intercept1 = mid_point1[0]

        if slope2 != float('inf'):
            intercept2 = mid_point2[1] - slope2 * mid_point2[0]
        else:
            intercept2 = mid_point2[0]

        # Paso 4: Resolver el sistema de ecuaciones para encontrar el centro
        if slope1 != slope2:
            center_x = (intercept2 - intercept1) / (slope1 - slope2)
            center_y = slope1 * center_x + intercept1
        else:
            # Si las mediatrices son paralelas, el centro es el punto medio entre los puntos medios
            center_x = (mid_point1[0] + mid_point2[0]) / 2
            center_y = (mid_point1[1] + mid_point2[1]) / 2

        # Paso 5: Verificar que el tercer punto esté en la circunferencia
        radius = ((point1[0] - center_x)**2 + (point1[1] - center_y)**2)**0.5

        # Devolver el centro y el radio
        return center_x, center_y, radius


    def get_centers(self, msg: PointCloud2):

        cones_all = point_cloud2.read_points(msg, field_names=("x", "y", "z","color", "score"), skip_nans=True)
        cones = np.array([[c[0],c[1]] for c in cones_all])



        max_inliers1=0
        best_center = [0,0]
        final_non_used_cones = None
        for _ in range(self.N_iterations):
            i,j,k = random.choices(range(len(cones)),k=3)
            x_center,y_center,r = self.find_circle_center(cones[i], cones[j], cones[k])

            if ((np.abs(r-7.625)<2) or (np.abs(r-10.625)<2)) and x_center>10:
                non_used_cones = np.full(cones.shape[0], True, dtype=np.bool_)
                inliers=0
                for i in range(cones.shape[0]):
                    d = np.linalg.norm(cones[i]-np.array([x_center,y_center]))
                    if np.abs(d-7.625)<self.threshold or np.abs(d-10.625)<self.threshold:
                        inliers+=1
                        non_used_cones[i]=False

                if inliers>max_inliers1:
                    best_center=[x_center,y_center]
                    max_inliers1=inliers
                    final_non_used_cones = non_used_cones

        max_inliers2=0
        best_second_center = best_center
        cones = cones[final_non_used_cones]
        if len(cones)>=3:
            for _ in range(self.N_iterations):
                i,j,k = random.choices(range(len(cones)),k=3)
                x_center,y_center,r = self.find_circle_center(cones[i],cones[j],cones[k])

                if ((np.abs(r-7.625)<2) or (np.abs(r-10.625)<2)) and x_center>10 and (np.abs(18.25 - np.linalg.norm([best_center[0]-x_center,best_center[1]-y_center])) < 2):
                    inliers=0
                    for i in range(cones.shape[0]):
                        d = np.linalg.norm(cones[i]-np.array([x_center,y_center]))
                        if (np.abs(d-7.625)<self.threshold or np.abs(d-10.625)<self.threshold):
                            inliers+=1

                    if inliers>max_inliers2:
                        best_second_center=[x_center,y_center]
                        max_inliers2=inliers

        rospy.loginfo([max_inliers1,max_inliers2])

        return best_center, best_second_center
       
        

        
