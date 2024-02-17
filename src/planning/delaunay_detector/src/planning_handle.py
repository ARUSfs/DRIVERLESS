"""Script to execute planning node

@author: Mariano del Río
@date: 20220504
"""

from itertools import combinations

from common_msgs.msg import Map, Trajectory
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from common_msgs.msg import Simplex, Triangulation
from scipy.interpolate import splprep, splev
import numpy as np

import rospy

from planning import PlanningSystem
from track_planning import TrackPlanningSystem


class PlanningHandle():
    """Listen map of cones, calculate center track via Delaunay
    triangulation and publish trajectory message.
    Every message received of cones, a new trajectory is published.
    """

    def __init__(self):

        self.planning_system = PlanningSystem()
        self.track_planning_system = TrackPlanningSystem()

        topic_perception_map = rospy.get_param('~topic_perception_map')
        rospy.Subscriber(topic_perception_map, Map, self.get_trajectory, queue_size=1)

        topic_global_map = rospy.get_param('~topic_global_map')
        rospy.Subscriber(topic_global_map, Map, self.get_global_track, queue_size=1)

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


    def get_global_track(self, msg):
        data = msg.cones  # List of cones
        self.track_planning_system.update_tracklimits(data)
        route = self.track_planning_system.calculate_path()
        route = route[1:]

        tck, u = splprep(route.T, s=0, per=True)  
        u_new = np.linspace(u.min(), u.max(), 1000)
        puntos_curva = np.array(splev(u_new, tck))

        acum=0
        s=[]
        s.append(0)
        xp = []
        yp = []
        for i in range(len(puntos_curva[0])-1):
            p1=puntos_curva[:,i]
            p2=puntos_curva[:,i+1]
            xp.append(p2[0]-p1[0])
            yp.append(p2[1]-p1[1])
            acum+=np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
            s.append(acum)
        xp.append(xp[-1])
        yp.append(yp[-1])

        xpp=[]
        ypp=[]
        for i in range(len(xp)-1):
            xpp.append(xp[i+1]-xp[i])
            ypp.append(yp[i+1]-yp[i])
        xpp.append(xpp[-1])
        ypp.append(xpp[-1])

        k=[]
        for i in range(len(xpp)):
            k.append((xp[i]*ypp[i] - xpp[i]*yp[i])/(xp[i]**2+yp[i]**2)**1.5)
        

        # k y s se pasarán al MPC !!!



