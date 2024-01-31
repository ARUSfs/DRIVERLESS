"""Script to execute planning node

@author: Mariano del Río
@date: 20220504
"""

from itertools import combinations

from common_msgs.msg import Map, Trajectory
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from common_msgs.msg import Simplex, Triangulation

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
        rospy.Subscriber(topic_map, Map, self.get_trajectory, queue_size=1)

        topic_route = rospy.get_param('~topic_route')
        self.pub_route = rospy.Publisher(topic_route, Trajectory, queue_size=1)

        topic_simplices = rospy.get_param('~topic_simplices')
        self.delaunay_publisher = rospy.Publisher(topic_simplices, Triangulation, queue_size=1)



    def get_trajectory(self, msg):
        data = msg.cones  # List of cones
        self.planning_system.update_tracklimits(data)
        self.publish_msg()

    def publish_msg(self):
        route, triang = self.planning_system.calculate_path()

        self.delaunay_publisher.publish(triang)

        msg = Trajectory()
        msg.trajectory = list()
        for midpoint in route:
            point = Point()
            point.x = midpoint[0]
            point.y = midpoint[1]
            point.z = 0
            msg.trajectory.append(point)

        self.pub_route.publish(msg)
