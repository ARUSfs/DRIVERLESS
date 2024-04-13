#!/usr/bin/env python3
import rospy
import csv
import os
from datetime import datetime
import rospkg
from fssim_common.msg import CarInfo, State

class DataLoggerNode:
    def __init__(self):
        # Obtener el path base usando rospkg
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('common_meta')

        # Definir el nombre de los archivos con marcas de tiempo
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.car_info_path = base_path+"/records/"+"car_info_"+timestamp+".csv"
        self.base_pose_path = base_path+"/records/"+"base_pose_"+timestamp+".csv"
        
        # Crear y abrir los archivos CSV
        self.car_info_csv = open(self.car_info_path, "w", newline='')
        self.car_info_writer = csv.writer(self.car_info_csv)
        self.car_info_writer.writerow(["Time", "Drag_Force_X", "Drag_Force_Y", "Drag_Force_Z", "Delta", "DC", "Front_Left_Steering_Angle", "Front_Right_Steering_Angle", "Delta_Measured", "Vx", "Vy", "R", "Torque_OK", "Alpha_F", "Alpha_F_Left", "Alpha_F_Right", "Alpha_R_Left", "Alpha_R", "Alpha_R_Right", "Fy_F", "Fy_F_Left", "Fy_F_Right", "Fy_R", "Fy_R_Left", "Fy_R_Right", "Fx"])
        
        self.base_pose_csv = open(self.base_pose_path, "w", newline='')
        self.base_pose_writer = csv.writer(self.base_pose_csv)
        self.base_pose_writer.writerow(["Time", "X", "Y", "Yaw", "Vx", "Vy", "R"])

        # Suscripciones a los tópicos
        self.car_info_sub = rospy.Subscriber("/fssim/car_info", CarInfo, self.car_info_callback)
        self.base_pose_sub = rospy.Subscriber("/fssim/base_pose_ground_truth", State, self.base_pose_callback)

    def car_info_callback(self, data):
        # Escribir la información recibida en el archivo CSV
        self.car_info_writer.writerow([rospy.Time.now()] + [getattr(data.drag_force.vec, dim) for dim in 'xyz'] + [getattr(data, attr) for attr in ('delta', 'dc', 'front_left_steering_angle', 'front_right_steering_angle', 'delta_measured', 'vx', 'vy', 'r', 'torque_ok', 'alpha_f', 'alpha_f_l', 'alpha_f_r', 'alpha_r_l', 'alpha_r', 'alpha_r_r', 'Fy_f', 'Fy_f_l', 'Fy_f_r', 'Fy_r', 'Fy_r_l', 'Fy_r_r', 'Fx')])

    def base_pose_callback(self, data):
        # Escribir la información recibida en el archivo CSV
        self.base_pose_writer.writerow([rospy.Time.now(), data.x, data.y, data.yaw, data.vx, data.vy, data.r])

    def close_files(self):
        # Cerrar los archivos CSV al terminar
        self.car_info_csv.close()
        self.base_pose_csv.close()

