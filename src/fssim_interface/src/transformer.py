"""
Script to execute fssim interface node

@author: Mariano del Río
@date: 20220524
"""

from common_msgs.msg import Map, Cone, CarState, Controls, Trajectory
from geometry_msgs.msg import Point, Polygon, PolygonStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from fssim_common.msg import State, Cmd

import rospy
import numpy as np


class transformerFssim():
    """
    Listen topics from Fssim and transform them to
    our topics and vice versa.
    """

    def __init__(self):

        self.subscribe_topics()
        self.pub_map = None
        self.pub_state = None
        self.pub_cmd = None
        self.pub_marker = None
        self.publish_topics()

    def publish_topics(self):

        self.pub_map = rospy.Publisher('perception_map', Map, queue_size=10)
        self.pub_state = rospy.Publisher('state', CarState, queue_size=10)
        self.pub_cmd = rospy.Publisher('/fssim/cmd', Cmd, queue_size=10)
        self.pub_marker = rospy.Publisher('/control/pure_pursuit/center_line',
                                          PolygonStamped, queue_size=10)

    def subscribe_topics(self):

        rospy.Subscriber('/camera/cones', PointCloud2, self.send_cones)
        rospy.Subscriber('fssim/base_pose_ground_truth', State,
                         self.send_state)
        rospy.Subscriber('controls', Controls, self.send_controllers)
        rospy.Subscriber('route', Trajectory, self.create_markers)

    def send_cones(self, msg):

        cones = point_cloud2.read_points(msg,
                                         field_names=("x", "y", "z",
                                                      "probability_blue",
                                                      "probability_yellow",
                                                      "probability_orange"),
                                         skip_nans=True)

        msg_map = Map()
        for c in cones:
            cone = Cone()
            cone.position.x = c[0]
            cone.position.y = c[1]
            prob = np.array(c[3:])

            # Index of color with highest probability
            maxp = np.where(prob == np.amax(prob))[0][0]

            if maxp == 0:  # blue
                cone.color = 'b'
                cone.confidence = prob[0]
            elif maxp == 1:  # yellow
                cone.color = 'y'
                cone.confidence = prob[1]
            if maxp == 2:  # orange
                cone.color = 'o'
                cone.confidence = prob[2]

            msg_map.cones.append(cone)

        rospy.loginfo(msg_map)
        self.pub_map.publish(msg_map)

    def send_state(self, msg):

        state = CarState()
        state.x = msg.x
        state.y = msg.y
        state.z = 0
        state.yaw = msg.yaw
        state.roll = 0
        state.pitch = 0
        state.vx = msg.vx
        state.vy = msg.vy
        state.vz = 0

        rospy.loginfo(state)
        self.pub_state.publish(state)

    def send_controllers(self, msg):

        cmd = Cmd()
        cmd.dc = msg.accelerator
        cmd.delta = msg.steering

        rospy.loginfo(cmd)
        self.pub_cmd.publish(cmd)

    def create_markers(self, msg):
        """
        Function to transform trajectory in message to draw
        in Fssim simulator and to send it.
        """

        data = msg.trajectory
        data = [Point(p.x, p.y, 0.0) for p in data]
        pol = Polygon(data)

        msg = PolygonStamped()
        msg.polygon = pol
        msg.header.frame_id = 'fssim/vehicle/cog'

        rospy.loginfo(msg)
        self.pub_marker.publish(msg)
