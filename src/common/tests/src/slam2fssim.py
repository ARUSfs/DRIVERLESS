#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import ros_numpy
from matplotlib import pyplot as plt
import numpy as np
import sys
import rospkg

rospack = rospkg.RosPack()
fssim_path = rospack.get_path('fssim_gazebo')+"/models/track/"

def pcl_callback(msg: PointCloud2):
    cones = point_cloud2.read_points(msg, field_names=("x", "y", "z","color", "score"), skip_nans=True)
    cones = np.array([[c[0],c[1]] for c in cones])
    cones_left = []
    cones_right = []
    for c in cones:
        if not rospy.is_shutdown():
            plt.scatter(-np.array(cones)[:,1],np.array(cones)[:,0])
            plt.scatter(-c[1],c[0])
            plt.axis("equal")
            plt.show(block=False)
            
            color = input("y: yellow, b: blue, else: not cone   ")
            if color == "y":
                cones_right.append(c)
            elif color == "b":
                cones_left.append(c)
            plt.close("all")

    if not rospy.is_shutdown():
        generate_files(cones_left,cones_right)
            

def generate_files(cones_left,cones_right):
    track_name = input("Track name: ")
    with open(fssim_path+track_name+".sdf","x") as file:
        file.write('<sdf version="1.4">\n<model name="some track">\n')
        for i,c in enumerate(cones_left):
            file.write("<include>\n<uri>model://fssim_gazebo/models/cone_blue</uri>\n<pose>"+str(c[0])+" "+str(c[1])+" 0 0 0 0</pose>\n<name>cone_left"+str(i)+"</name>\n</include>\n")
        for i,c in enumerate(cones_right):
            file.write("<include>\n<uri>model://fssim_gazebo/models/cone_yellow</uri>\n<pose>"+str(c[0])+" "+str(c[1])+" 0 0 0 0</pose>\n<name>cone_right"+str(i)+"</name>\n</include>\n")
        file.write("<include>\n<uri>model://fssim_gazebo/models/time_keeping</uri>\n<pose>6.0 3.0 0 0 0 0</pose>\n<name>tk_device_0</name>\n</include>\n<include>\n<uri>model://fssim_gazebo/models/time_keeping</uri>\n<pose>6.0 -3.0 0 0 0 0</pose>\n<name>tk_device_1</name>\n</include>")
        file.write('</model>\n</sdf>')


    with open(fssim_path+"tracks_yaml/"+track_name+".yaml","x") as file:
        file.write("cones_left:\n")
        for c in cones_left:
            file.write("- - "+str(c[0])+"\n  - "+str(c[1])+"\n")
        file.write("cones_orange: []\ncones_orange_big: []\n")
        file.write("cones_right:\n")
        for c in cones_right:
            file.write("- - "+str(c[0])+"\n  - "+str(c[1])+"\n")
        
        file.write("starting_pose_front_wing:\n- 0.0\n- 0.0\n- 0.0\ntk_device:\n- - 6.0\n  - 3.0\n- - 6.0\n  - -3.0")




if __name__ == '__main__':
    rospy.init_node('track_generator', anonymous=True)
    r = rospy.Rate(0.5)
    msg = rospy.wait_for_message('/global_map', PointCloud2)
    pcl_callback(msg)