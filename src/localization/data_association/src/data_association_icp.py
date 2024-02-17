'''
@Author: Carlos Pérez Cantalapiedra
@Description: Data association for mapping using ICP.
@Date: 2023-12-27
@LastUpdate: 2024-02-15
'''

import numpy as np
import open3d as o3d
import rospy
from fssim_common.msg import State
from common_msgs.msg import Map, Cone
from visualization_msgs.msg import MarkerArray, Marker
from scipy.interpolate import splprep, splev

# Constants
min_dist = rospy.get_param('/data_association/min_dist')
max_dist = rospy.get_param('/data_association/max_dist')
new_lap_counter = rospy.get_param('/data_association/new_lap_counter')

class Data_association2:

    def __init__(self):

        # Definiendo suscribers y publishers
        rospy.Subscriber('/fssim/base_pose_ground_truth', State, self.state_callback, queue_size=20)
        rospy.Subscriber('/perception_map', Map, self.camera_callback, queue_size=1)
        self.pMap = rospy.Publisher('/map', Map, queue_size=1)
        self.pMapView = rospy.Publisher('/landmarks', MarkerArray, queue_size=1)


        # Inicializando variables
        self.map = Map() #Mapa
        self.state = np.zeros(3) # Estado del coche [x, y, yaw]
        self.measurements = np.empty((0,2)) # Mediciones del sensor
        self.u = np.zeros(2) # Inputs de control [v, yaw]
        self.empty_map = True # Indica si el mapa está vacío
        self.count = 0
        self.enable_mapping = True #Habilita el mapeado
        self.measurement_points_ICP= np.empty((0,2)) # Mediciones usadas para el ICP
        self.map_points_ICP = np.empty((0,2)) # Puntos del mapa usados para el ICP
        self.path = np.array([[0.,0.]])
        self.last_pos = np.array([0.,0.])
    
    
    def state_callback(self, msg: State):

        # Se actualizan los inputs de control
        self.u[0] = np.linalg.norm([msg.vx, msg.vy])-0.010
        self.u[1] = msg.r

        # Se actualiza el estado del coche
        self.state[0] = msg.x
        self.state[1] = msg.y
        self.state[2] = msg.yaw


    def camera_callback(self, msg: Map):

        if (np.linalg.norm(self.state[:2]-self.last_pos) > 2):
            self.last_pos = np.array([[self.state[0], self.state[1]]])
            self.path = np.vstack((self.path, [self.state[0], self.state[1]]))
        
        if self.enable_mapping:
            # Se obtienen las mediciones del sensor
            points = self.map_to_xyz_array(msg)
            self.measurements = np.copy(points)

            # Si el mapa está vacío se añaden las mediciones
            if self.empty_map == True:
                m = self.measurements.copy()
                yaw = self.state[2]
                rot = np.array([[np.cos(yaw), -np.sin(yaw)],
                        [np.sin(yaw), np.cos(yaw)]])
                x = self.state[0]
                y = self.state[1]
                for i in m:
                    corrected_position = rot @ i + np.array([x, y])
                    self.map.cones.append(self.new_cone(corrected_position[0], corrected_position[1], 'o'))
                self.empty_map = False 
                
            # Si el mapa no está vacío se pasan los landmarks al marco global y se realiza la asociación de datos
            else:
                self.data_association()
            
            # Se comprueba si se ha completado una vuelta
            if self.count > new_lap_counter and self.u[0] > 0.2:
                rospy.loginfo("%s", self.path)
                self.coloring()
                self.pMap.publish(self.map)
                self.pub_markers()
                self.map = Map()
                self.count = 0
                self.path = np.array([[0.,0.]])
                self.empty_map= True
                
            # Se publica el mapa y su vista
            if len(self.map.cones) > 0:
                self.pub_markers()
        
        else:
            pass

            
    
    def data_association(self):

        c = 0
        x = self.state[0]
        y = self.state[1]
        yaw = self.state[2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)],
                        [np.sin(yaw), np.cos(yaw)]])
        measurement_points_ICP= np.empty((0,2)) # Mediciones usadas para el ICP
        map_points_ICP = np.empty((0,2)) # Puntos del mapa usados para el ICP
        new_landmarks_without_correction = np.empty((0,2)) # Landmarks que no se han podido asociar antes de ser corregidos por el ICP
        
        for measurement in self.measurements:
            new_landmark = True
            gM = rot @ measurement + np.array([x, y])

            for l in self.map.cones:

                landmark = np.array([l.position.x, l.position.y])
                
                if np.linalg.norm(gM - landmark) < min_dist :
                    measurement_points_ICP = np.vstack((self.measurement_points_ICP, gM))
                    map_points_ICP = np.vstack((self.map_points_ICP, landmark))
                    new_landmark = False
                    break

            if new_landmark and np.linalg.norm(gM-self.state[:2]) < max_dist: 
                new_landmarks_without_correction = np.vstack((new_landmarks_without_correction, gM))
                c += 1
        

        # Se añaden los landmarks que no se han podido asociar aplicandoles el ICP
        transformation_matrix = self.icp(measurement_points_ICP,map_points_ICP)
        self.landmarks_correction(new_landmarks_without_correction, transformation_matrix)

        if c == 0 and self.u[0] > 0.2:
            self.count += 1

        else:
            self.count = 0
    
    
    def icp(self,s,t):
        #Aplica ICP y devuelve la matriz de transformación
        num_map_points = t.shape[0]
        num_lidar_points = s.shape[0]
        map_cloud = o3d.geometry.PointCloud()
        measurement_cloud = o3d.geometry.PointCloud()
        map_cloud.points = o3d.utility.Vector3dVector(np.hstack((t, np.zeros((num_map_points, 1)))))
        measurement_cloud.points = o3d.utility.Vector3dVector(np.hstack((s, np.zeros((num_lidar_points, 1)))))
        transformation_matrix = o3d.pipelines.registration.registration_icp(
                                                    source = measurement_cloud,
                                                    target = map_cloud,
                                                    max_correspondence_distance = 0.1,
                                                    estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                    ).transformation
        return transformation_matrix
    
    
    def landmarks_correction(self, new_landmarks_without_correction, transformation_matrix):
        
        #Se aplica la matriz de transformacion a los landmarks antes de añadirlos al mapa
        r = transformation_matrix[:2,:2]
        t = transformation_matrix[:2,2]
        lc = new_landmarks_without_correction @ r.T + t
        for i in lc:
            self.map.cones.append(self.new_cone(i[0], i[1], 'o'))

    
    def new_cone(self, x, y, color):
        c = Cone()
        c.position.x = x
        c.position.y = y
        c.position.z = 0
        c.color = color
        return c
    

    def coloring(self):
        #Colorea los conos según si están dentro o fuera de la curva generada por el recorrido del coche.
        tck, u = splprep(self.path.T, s=0, per=True)
        u_new = np.linspace(u.min(), u.max(), 1000)
        puntos_curva = np.array(splev(u_new, tck))
        
        for c in self.map.cones:
            n_cortes = 0
            x0 = c.position.x
            y0 = c.position.y
            
            for i in range(len(puntos_curva[0]) - 1):
                punto1 = puntos_curva[:, i]
                punto2 = puntos_curva[:, i + 1]
                
                if ((punto1[0] < x0 and punto2[0] > x0) or (punto1[0] > x0 and punto2[0] < x0)) and punto1[1]>y0 :
                    n_cortes += 1
            
            if n_cortes % 2 == 0:
                c.color = 'b'
            
            else:
                c.color = 'y'
            
    
    def map_to_xyz_array(self, m : Map):
        points = np.empty((0,2))
        for c in m.cones:
            points = np.vstack((points, np.array([[c.position.x, c.position.y]])))
        return points


    def pub_markers(self):
        marray = MarkerArray()
        m1 = Marker()
        m1.id = 100000
        m1.action = Marker.DELETEALL
        marray.markers.append(m1)
        for i, lm in enumerate(self.map.cones):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time().now()
            marker.type = Marker.CYLINDER
            marker.action = Marker.MODIFY
            marker.pose.position.x = lm.position.x
            marker.pose.position.y = lm.position.y
            marker.pose.position.z = lm.position.z
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            if lm.color == 'b':
                marker.color.r = 0
                marker.color.g = 0
                marker.color.b = 1
                marker.color.a = 1.0
            elif lm.color == 'y':
                marker.color.r = 1
                marker.color.g = 1
                marker.color.b = 0
                marker.color.a = 1.0
            else:
                marker.color.r = 0
                marker.color.g = 0
                marker.color.b = 0
                marker.color.a = 1.0

            marker.lifetime = rospy.Duration()

            marray.markers.append(marker)
            marker.lifetime = rospy.Duration()
            marray.markers.append(marker)

        self.pMapView.publish(marray)