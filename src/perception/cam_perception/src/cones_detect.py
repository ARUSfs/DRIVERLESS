#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from common_msgs.msg import Cone
from darknet import darknet
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray,Marker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Cone_detect():
    
    def __init__(self):
        self.sub = rospy.Subscriber("/cam/image_raw", Image, self.detecta, queue_size=1)
        self.pub = rospy.Publisher("/cam/cones", Cone, queue_size=15)                   #Publisher de los conos detectados
        self.pub2 = rospy.Publisher("/cam_marker",MarkerArray,queue_size=1)             #Publisher de los markers
        self.pub4 = rospy.Publisher("/cam/image_rviz",Image,queue_size=1)               #Publisher de la imagen
        path = rospy.get_param("/cam_detect/path")                                      #Ruta del equipo
        mat_hom = path + '/DRIVERLESS/src/perception/cam_perception/data/mathom.txt'    #Ruta de la matriz de homografía
        mat_int = path + '/DRIVERLESS/src/perception/cam_perception/data/matint.txt'    #Ruta de la matriz intrínseca
        dist_path = path + '/DRIVERLESS/src/perception/cam_perception/data/dist.txt'    #Ruta del archivo de distorsión
        self.hom = np.loadtxt(mat_hom)                                                  #Matriz de homografía
        self.int = np.loadtxt(mat_int)                                                  #Matriz intrínseca
        self.dist = np.loadtxt(dist_path)                                               #Matriz de distorsión
        self.bridge = CvBridge()
        
        #YOLO
        self.cfg = path + '/DRIVERLESS/src/perception/cam_perception/weights/cones-customanchors.cfg'
        self.data = path + '/DRIVERLESS/src/perception/cam_perception/weights/cones.data'
        self.weights = path + '/DRIVERLESS/src/perception/cam_perception/weights/cones5.weights'
        self.net, self.names, self.colors = darknet.load_network(self.cfg, self.data, self.weights)
        self.w_net, self.h_net = darknet.network_width(self.net), darknet.network_height(self.net)

        
    def yolo_detect(self, img: np.ndarray):
        #frame = cv2.resize(img, (self.w_net, self.h_net))
        d_img = darknet.make_image(self.w_net, self.h_net, 3) 
        darknet.copy_image_from_bytes(d_img,img.tobytes())
        detections = darknet.detect_image(self.net, self.names, d_img)
        return detections

    def cones_perception(self, detection: list):  
        cones_detected_info = []
        for label,confidence,box in detection:
            xmin,ymin,xmax,_ = darknet.bbox2points(box)
            mid = xmin + ((xmax-xmin) / 2)
            cones_detected_info.append((label, confidence, [mid, ymin,1])) #añado 1 para cuadrar dimensiones con la matriz de la homografía
        return cones_detected_info

    def colorea(self,color_cone):
        """Función que devuelve el color del cono detectado en el formato de la clase Cone"""
        if color_cone == 'blue_cone':
            col = 'b'
        elif color_cone == 'yellow_cone':
            col = 'y'
        else:
            col = 'o'
        return col

    def detecta(self, frame):
        img_cv2 = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        img_res = cv2.resize(img_cv2, (self.w_net, self.h_net))
        img_und = cv2.undistort(img_res, self.int, self.dist) # Probar los colores de la imagen!!! 
        img = cv2.cvtColor(img_und, cv2.COLOR_BGR2RGB)      # Quitar y renombrar si no es necesario!!!
        detections = self.yolo_detect(img)
        cones_detected_info = self.cones_perception(detections)            
    #Mostrar img en rviz
        img_msg = self.bridge.cv2_to_imgmsg(img_und, encoding="bgr8")
        self.pub4.publish(img_msg)
        MA = MarkerArray()
        ind=0
        for (tipo, segur, box) in cones_detected_info:
            if float(segur)>80:                  
                array = np.array(box)    
                #Publica los conos homografiados  
                coord = self.hom @ array
                coord /= coord[2]
                cone = Cone()
                cone.position = Point(coord[0], coord[1], 0.32)
                cone.color = self.colorea(tipo)
                cone.confidence = np.float64(segur)
                self.pub.publish(cone)
                                            
                #Mostrar conos en rviz
                M = Marker()
                M.id = ind
                M.header.frame_id = "map"
                M.type = Marker.CYLINDER
                M.action = Marker.ADD
                M.scale.x = 0.2
                M.scale.y = 0.2
                M.scale.z = 0.5
                M.color.a = 1
                if cone.color=='y':
                    M.color.r = 1.0
                    M.color.g = 1.0
                    M.color.b = 0.0
                elif cone.color=='b':
                    M.color.r = 0.0
                    M.color.g = 0.0
                    M.color.b = 1.0
                else:
                    M.color.r = 1.0
                    M.color.g = 0.0
                    M.color.b = 0.0
                M.pose.position.x = cone.position.x
                M.pose.position.y = cone.position.y
                M.pose.position.z = 0
                MA.markers.append(M)
                ind +=1
        self.pub2.publish(MA)


class Video_stream():
    def __init__(self):
        self.pub = rospy.Publisher("/cam/image_raw", Image, queue_size=1)
        self.cam = rospy.get_param("/cam_detect/cam")
        self.vid = cv2.VideoCapture(self.cam)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)

    def stream(self):
        while not rospy.is_shutdown():
            ret, frame = self.vid.read()
            if ret:
                img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.pub.publish(img)
            self.rate.sleep()
        self.vid.release()

