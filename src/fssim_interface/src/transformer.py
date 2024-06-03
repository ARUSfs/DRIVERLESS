"""
Script to execute fssim interface node

@author: Mariano del Río
@date: 20220524
"""

from common_msgs.msg import CarState, Controls, Trajectory
from geometry_msgs.msg import Point, Polygon, PolygonStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from fssim_common.msg import State, Cmd
from visualization_msgs.msg import Marker
from std_msgs.msg import Int16

import rospy
import numpy as np
import math



class transformerFssim():
    """
    Listen topics from Fssim and transform them to
    our topics and vice versa.
    """

    def __init__(self):

        self.pub_map = None
        self.pub_state = None
        self.pub_cmd = None
        self.pub_as_status = None
        self.publish_topics()
        self.subscribe_topics()
        

    def publish_topics(self):

        self.pub_map = rospy.Publisher('/perception_map', PointCloud2, queue_size=10)
        self.pub_state = rospy.Publisher('/car_state/state', CarState, queue_size=10)
        self.pub_cmd = rospy.Publisher('/fssim/cmd', Cmd, queue_size=10)
        self.pub_as_status = rospy.Publisher('/can/AS_status', Int16, queue_size=10)

    def subscribe_topics(self):

        rospy.Subscriber('/lidar/cones', PointCloud2, self.send_cones)
        rospy.Subscriber('/controls', Controls, self.send_controllers)
        rospy.Timer(rospy.Duration(1), self.GOsignal_callback)

    def send_cones(self, msg):

        cones = point_cloud2.read_points(msg,
                                         field_names=("x", "y", "z",
                                                      "probability_blue",
                                                      "probability_yellow",
                                                      "probability_orange"),
                                         skip_nans=True)
        
        points = []
        for c in cones:
            prob = np.array(c[3:])

            # Index of color with highest probability
            maxp = np.where(prob == np.amax(prob))[0][0]

            if maxp == 0:  # blue
                points.append([c[0],c[1],c[2],0,prob[0]])
            elif maxp == 1:  # yellow
                points.append([c[0],c[1],c[2],1,prob[1]])
            if maxp == 2:  # orange
                points.append([c[0],c[1],c[2],2,prob[2]])

        fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="color", offset=12, datatype=PointField.UINT32, count=1),
        PointField(name="score", offset=16, datatype=PointField.FLOAT32, count=1)
        ]
        new_cloud = point_cloud2.create_cloud(header=msg.header, fields=fields,points=points)
        new_cloud.is_dense = True
        
        self.pub_map.publish(new_cloud)


    def send_controllers(self, msg):

        cmd = Cmd()
        cmd.dc = msg.accelerator
        cmd.delta = math.radians(msg.steering)

        rospy.loginfo(cmd)
        self.pub_cmd.publish(cmd)
        

    
    def GOsignal_callback(self,msg):
        status_msg=Int16()
        status_msg.data=0x02
        self.pub_as_status.publish(status_msg)
