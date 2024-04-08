#!/usr/bin/env python3

import rospy
import csv
from fssim_common.msg import CarInfo, State
class DataLoggerNode():
    def __init__(self):

        # Open CSV files for writing
        self.car_info_csv = open("car_info.csv", "w", newline='')
        self.car_info_writer = csv.writer(self.car_info_csv)
        self.car_info_csv.truncate(0)
        self.car_info_writer.writerow(["Time", "Drag_Force_X", "Drag_Force_Y", "Drag_Force_Z", "Delta", "DC", "Front_Left_Steering_Angle", "Front_Right_Steering_Angle", "Delta_Measured", "Vx", "Vy", "R", "Torque_OK", "Alpha_F", "Alpha_F_Left", "Alpha_F_Right", "Alpha_R_Left", "Alpha_R", "Alpha_R_Right", "Fy_F", "Fy_F_Left", "Fy_F_Right", "Fy_R", "Fy_R_Left", "Fy_R_Right", "Fx"])
        
        self.base_pose_csv = open("base_pose_ground_truth.csv", "w", newline='')
        self.base_pose_writer = csv.writer(self.base_pose_csv)
        self.base_pose_csv.truncate(0)
        self.base_pose_writer.writerow(["Time", "X", "Y", "Yaw", "Vx", "Vy", "R"])

        # Subscriber for car_info
        self.car_info_sub = rospy.Subscriber("/fssim/car_info", CarInfo, self.car_info_callback)
        
        # Subscriber for base_pose_ground_truth
        self.base_pose_sub = rospy.Subscriber('/fssim/base_pose_ground_truth', State, self.base_pose_callback)

    def car_info_callback(self, data):
        row = [
            rospy.Time.now(),
            data.drag_force.vec.x,
            data.drag_force.vec.y,
            data.drag_force.vec.z,
            data.delta,
            data.dc,
            data.front_left_steering_angle,
            data.front_right_steering_angle,
            data.delta_measured,
            data.vx,
            data.vy,
            data.r,
            data.torque_ok,
            data.alpha_f,
            data.alpha_f_l,
            data.alpha_f_r,
            data.alpha_r_l,
            data.alpha_r,
            data.alpha_r_r,
            data.Fy_f,
            data.Fy_f_l,
            data.Fy_f_r,
            data.Fy_r,
            data.Fy_r_l,
            data.Fy_r_r,
            data.Fx
        ]
        self.car_info_writer.writerow(row)

    def base_pose_callback(self, data):
        row = [
            rospy.Time.now(),
            data.x,
            data.y,
            data.yaw,
            data.vx,
            data.vy,
            data.r
        ]
        self.base_pose_writer.writerow(row)

def main():
    rospy.init_node('DataLoggerNode', anonymous=True)
    DataLoggerNode()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
