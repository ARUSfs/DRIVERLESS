import numpy as np
import open3d as o3d
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from fssim_common.msg import State
from common_msgs.msg import Map, Cone
from visualization_msgs.msg import MarkerArray, Marker
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array

min_dist = 1.8 # Distancia máxima para asociar un landmark
max_dist = 8 # Distancia máxima para añadir un landmark
new_lap_counter = 30 # camara callbacks sin añadir landmarks para considerar que se ha completado una vuelta

class Data_association2:

    def __init__(self):

        # Definiendo suscribers y publishers
        rospy.Subscriber('/fssim/base_pose_ground_truth', State, self.state_callback, queue_size=20)
        rospy.Subscriber('/camera/cones', PointCloud2, self.camera_callback, queue_size=1)
        self.pMap = rospy.Publisher('/map', Map, queue_size=1)
        self.pMapView = rospy.Publisher('/landmarks', MarkerArray, queue_size=1)


        # Inicializando variables
        self.map = np.empty((0,2)) # Mapa de landmarks
        self.state = np.zeros(3) # Estado del coche [x, y, yaw]
        self.measurements = np.empty((0,2)) # Mediciones del sensor
        self.u = np.zeros(2) # Inputs de control [v, yaw]
        self.empty_map = True # Indica si el mapa está vacío
        self.count = 0
        self.enable_mapping = True
        self.measurement_points_ICP= np.empty((0,2)) # Mediciones usadas para el ICP
        self.map_points_ICP = np.empty((0,2)) # Puntos del mapa usados para el ICP
    
    
    def state_callback(self, msg: State):

        # Se actualizan los inputs de control
        self.u[0] = np.linalg.norm([msg.vx, msg.vy])-0.010
        self.u[1] = msg.r

        # Se actualiza el estado del coche
        self.state[0] = msg.x
        self.state[1] = msg.y
        self.state[2] = msg.yaw


    def camera_callback(self, msg: PointCloud2):
        
        if self.enable_mapping:
            # Se obtienen las mediciones del sensor
            points = pointcloud2_to_xyz_array(msg)
            points[:, 2] = 1
            self.measurements = np.copy(points[:, :2])

            # Si el mapa está vacío se añaden las mediciones
            if self.empty_map == True:
                self.map = self.measurements.copy()
                self.empty_map = False 
                
            # Si el mapa no está vacío se pasan los landmarks al marco global y se realiza la asociación de datos
            else:
                self.data_association()
            
            # Se comprueba si se ha completado una vuelta
            if self.count > new_lap_counter and self.u[0] > 0.2:
                self.count = 0
                self.enable_mapping = False
                rospy.loginfo('Lap completed')
                rospy.loginfo('Number of landmarks: ' + str(self.map.shape[0]))
                rospy.loginfo('Map: ' + str(self.map))
                
            # Se publica el mapa y su vista
            map_msg = Map()
            for landmark in self.map:
                cone = Cone()
                p = Point()
                p.x = landmark[0]
                p.y = landmark[1]
                p.z = 0
                cone.position = p
                cone.color = 'b'
                cone.confidence = 1
                map_msg.cones.append(cone)
            self.pMap.publish(map_msg)
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

            for landmark in self.map:
                
                if np.linalg.norm(gM - landmark) < min_dist :
                    measurement_points_ICP = np.vstack((self.measurement_points_ICP, gM))
                    map_points_ICP = np.vstack((self.map_points_ICP, landmark))
                    new_landmark = False
                    break

            if new_landmark and np.linalg.norm(gM-self.state[:2]) < max_dist: 
                new_landmarks_without_correction = np.vstack((new_landmarks_without_correction, gM))
                c += 1
        
        transformation_matrix = self.icp(measurement_points_ICP,map_points_ICP)
        self.landmarks_correction(new_landmarks_without_correction, transformation_matrix)
        
        if c == 0 and self.u[0] > 0.2:
            self.count += 1

        else:
            self.count = 0
    
    
    def icp(self,s,t):
        #Aplica ICMP y devuelve la matriz de transformación
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
        self.map = np.vstack((self.map, new_landmarks_without_correction @ r.T + t))


    def pub_markers(self):
        marray = MarkerArray()
        m1 = Marker()
        m1.action = Marker.DELETEALL
        marray.markers.append(m1)
        for i, lm in enumerate(self.map):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time().now()
            marker.type = Marker.CYLINDER
            marker.action = Marker.MODIFY
            marker.pose.position.x = lm[0]
            marker.pose.position.y = lm[1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1.0

            marker.lifetime = rospy.Duration()

            marray.markers.append(marker)
            marker.lifetime = rospy.Duration()
            marray.markers.append(marker)

        self.pMapView.publish(marray)