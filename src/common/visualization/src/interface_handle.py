"""Script to send data to RVIZ Interface

@author: Mariano del Río
@date: 20221019
"""
from itertools import combinations

from common_msgs.msg import Map, Trajectory, Triangulation
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Point

import rospy


class InterfaceHandle():

    def __init__(self):
        self.pub = None
        self.pub2 = None
        self.pub3 = None
        self.cones = []
        self.frame = rospy.get_param("/interface/frame")
        self.publish_topics()
        self.subscribe_topics()

    def subscribe_topics(self):
        # map_topic = rospy.get_param("/interface/topic_map")
        # rospy.Subscriber(map_topic, Map, self.read_map)

        topic_route = rospy.get_param("/interface/topic_route")
        rospy.Subscriber(topic_route, Trajectory, self.read_route)

        topic_pursuit = rospy.get_param("/interface/topic_pursuit")
        rospy.Subscriber(topic_pursuit, Point, self.read_pursuit_point)

        topic_delaunay = rospy.get_param("/interface/topic_delaunay")
        rospy.Subscriber(topic_delaunay, Triangulation, self.delaunay_callback)

    def publish_topics(self):
        # self.pub = rospy.Publisher("visualization_cones", MarkerArray, queue_size=1)
        self.pub2 = rospy.Publisher("visualization_path", Marker, queue_size=1)
        self.pub3 = rospy.Publisher("visualization_pursuit", Marker, queue_size=1)
        self.delaunay_pub = rospy.Publisher("visualization_delaunay", Marker, queue_size=1)



    # def read_map(self, msg):
    #     data = msg.cones  # List of cones
    #     cones = list()
    #     for c in data:
    #         cone = (c.position.x, c.position.y, c.color, c.confidence)
    #         cones.append(cone)

    #     self.cones = cones
    #     self.draw(cones)

    # def draw(self, cones):
    #     markerarray = MarkerArray()
    #     m1 = Marker()
    #     m1.action = Marker.DELETEALL
    #     markerarray.markers.append(m1)
    #     for i, p in enumerate(cones):
    #         px, py, c, prob = p
    #         marker = self.create_marker(px, py, c, i)
    #         markerarray.markers.append(marker)

    #     self.pub.publish(markerarray)

    def create_marker(self, px, py, c, i):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = rospy.Time().now()

        marker.ns = "cone"
        marker.id = i
        marker.type = Marker.CYLINDER
        marker.action = Marker.MODIFY

        marker.pose.position.x = px
        marker.pose.position.y = py
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        if c == 'b':
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 255
            marker.color.a = 1.0
        elif c == 'y':
            marker.color.r = 255
            marker.color.g = 255
            marker.color.b = 0.0
            marker.color.a = 1.0
        elif c == 'o':
            marker.color.r = 180
            marker.color.g = 180
            marker.color.b = 0.0
            marker.color.a = 1.0

        marker.lifetime = rospy.Duration()
        return marker



    def read_route(self, msg):
        data = msg.trajectory  # List of points
        points = [[p.x, p.y] for p in data]
        self.send_route(points)

    def send_route(self, points):
        path_marker = Marker()
        path_marker.header.frame_id = self.frame
        path_marker.ns = 'delaunay'
        path_marker.header.stamp = rospy.Time.now()
   
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.MODIFY
        path_marker.scale.x = 0.1
        path_marker.color.a = 1.0
        path_marker.color.r = 1.0

        path_marker.points = list()
        for p in points:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            path_marker.points.append(point)

        self.pub2.publish(path_marker)



    def read_pursuit_point(self, msg):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = rospy.Time().now()

        marker.ns = "pursuit"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY

        marker.pose.position.x = msg.x
        marker.pose.position.y = msg.y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 255
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()
        self.pub3.publish(marker)



    def delaunay_callback(self, triang):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'delaunay'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.MODIFY
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.points = list()
        for s in triang.simplices:
            for p, q in combinations(s.simplex, 2):
                marker.points.append(p)
                marker.points.append(q)

        self.delaunay_pub.publish(marker)
