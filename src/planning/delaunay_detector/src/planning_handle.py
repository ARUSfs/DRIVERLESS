"""Script to execute planning node

@author: Mariano del Río
@date: 20220504
"""

from common_msgs.msg import Map
from common_msgs.msg import Trajectory
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

        self.subscribe_topics()
        self.pub_route = None
        self.publish_topics()

    def subscribe_topics(self):
        topic1 = rospy.get_param('/delaunay_detector/topic_map')
        rospy.Subscriber(topic1, Map, self.get_trajectory)

    def publish_topics(self):
        topic2 = rospy.get_param('/delaunay_detector/topic_route')
        self.pub_route = rospy.Publisher(topic2, Trajectory,
                                         queue_size=10)

    def get_trajectory(self, msg):

        data = msg.cones  # List of cones
        self.planning_system.update_tracklimits(data)
        self.publish_msg()

    def publish_msg(self):

        route, weight = self.planning_system.calculate_path()

        msg = self.transform_to_msg_trajectory(route)
        rospy.loginfo(msg)
        rospy.loginfo(weight)
        self.pub_route.publish(msg)

    def transform_to_msg_trajectory(self, route: list):

        msg = Trajectory()
        points = []
        for p in route:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = 0
            points.append(point)

        msg.trajectory = points
        return msg
