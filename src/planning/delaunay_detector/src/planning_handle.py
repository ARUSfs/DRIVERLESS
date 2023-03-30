"""Script to execute planning node

@author: Mariano del Río
@date: 20220504
"""

from itertools import combinations

from common_msgs.msg import Map, Trajectory
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import rospy

from planning import PlanningSystem


class PlanningHandle():
    """Listen map of cones, calculate center track via Delaunay
    triangulation and publish trajectory message.
    Every message received of cones, a new trajectory is published.
    """

    def __init__(self):

        self.planning_system = PlanningSystem()

        topic_map = rospy.get_param('~topic_map')
        rospy.Subscriber(topic_map, Map, self.get_trajectory)

        topic_route = rospy.get_param('~topic_route')
        self.pub_route = rospy.Publisher(topic_route, Trajectory, queue_size=1)

        self.bpublish_simplices = rospy.get_param('~send_simplices_marker')
        if self.bpublish_simplices:
            topic_route = rospy.get_param('~topic_simplices')
            self.pub_marker = rospy.Publisher(topic_route, Marker, queue_size=1)



    def get_trajectory(self, msg):
        data = msg.cones  # List of cones
        self.planning_system.update_tracklimits(data)
        self.publish_msg()

    def publish_msg(self):
        route, simplices = self.planning_system.calculate_path()
        if simplices is not None and len(simplices) > 0 and self.bpublish_simplices:
            self.publish_marker(simplices)
        msg = Trajectory()
        msg.trajectory = list()
        for midpoint in route:
            point = Point()
            point.x = midpoint[0]
            point.y = midpoint[1]
            point.z = 0
            msg.trajectory.append(point)

        rospy.loginfo(msg)
        self.pub_route.publish(msg)

    def publish_marker(self, simplices):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()

        marker.ns = 'delaunay'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.MODIFY
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.points = list()
        cones = self.planning_system.cones
        for simplex in simplices:
            for a, b in combinations(simplex, 2):
                point_a = Point()
                point_a.x = cones[a, 0]
                point_a.y = cones[a, 1]
                marker.points.append(point_a)

                point_b = Point()
                point_b.x = cones[b, 0]
                point_b.y = cones[b, 1]
                marker.points.append(point_b)

        self.pub_marker.publish(marker)
