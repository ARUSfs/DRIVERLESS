"""Script to execute planning node

@author: Mariano del Río
@date: 20220504
"""

from itertools import combinations
from common_msgs.msg import Trajectory, CarState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from common_msgs.msg import Simplex, Triangulation
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
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

        self.path = np.array([[0,0]])

        self.planning_system = PlanningSystem()
        self.track_planning_system = TrackPlanningSystem()

        topic_perception_map = rospy.get_param('~topic_perception_map')
        rospy.Subscriber(topic_perception_map, PointCloud2, self.get_trajectory, queue_size=1)

        topic_global_map = rospy.get_param('~topic_global_map')
        rospy.Subscriber(topic_global_map, PointCloud2, self.get_global_track, queue_size=1)

        topic_route = rospy.get_param('~topic_route')
        self.pub_route = rospy.Publisher(topic_route, Trajectory, queue_size=1)

        topic_simplices = rospy.get_param('~topic_simplices')
        self.delaunay_publisher = rospy.Publisher(topic_simplices, Triangulation, queue_size=1)

        self.pub_global_route = rospy.Publisher('/delaunay/global_route', Trajectory, queue_size=1)

        rospy.Subscriber('/car_state/state', CarState, self.accum_path, queue_size=1)
        self.colored_map_pub = rospy.Publisher("/global_colored_map", PointCloud2, queue_size=1)

    def get_trajectory(self, msg):
        cones = point_cloud2.read_points(msg, field_names=("x", "y", "z","color","score"),skip_nans=True)
        self.planning_system.update_tracklimits(cones)
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


    def get_global_track(self,msg :PointCloud2):
        colored_cones = self.coloring(msg)
        self.track_planning_system.update_tracklimits(colored_cones)
        route = self.track_planning_system.calculate_path()

        route = route[1:]
        
        # rospy.logwarn(route)
        msg2 = Trajectory()
        msg2.trajectory = [Point(p[0],p[1],0) for p in route]
        # rospy.logwarn(msg2)
        rospy.logerr([len(route),len(msg2.trajectory)])

        self.pub_global_route.publish(msg2)

    def accum_path(self, msg:CarState):
        if np.linalg.norm(self.path[-1]-np.array([msg.x,msg.y]))>4 :
            self.path = np.vstack((self.path, [msg.x, msg.y]))

    def coloring(self,msg: PointCloud2):
        #Colorea los conos según si están dentro o fuera de la curva generada por el recorrido del coche.
        map = point_cloud2.read_points(msg, field_names=("x", "y", "z","color","score"),skip_nans=True)
        
        
        tck, u = splprep(self.path.T, s=0, per=True)
        u_new = np.linspace(u.min(), u.max(), 1000)
        puntos_curva = np.array(splev(u_new, tck))
        colored_map = []

        for c in map: 
            n_cortes = 0
            x0 = c[0]
            y0 = c[1]
            
            for i in range(len(puntos_curva[0]) - 1):
                punto1 = puntos_curva[:, i]
                punto2 = puntos_curva[:, i + 1]
                
                if ((punto1[0] < x0 and punto2[0] > x0) or (punto1[0] > x0 and punto2[0] < x0)) and punto1[1]>y0 :
                    n_cortes += 1
            

            if n_cortes % 2 == 0:  # blue
                colored_map.append([c[0],c[1],c[2],0,c[4]])
            else:  # yellow
                colored_map.append([c[0],c[1],c[2],1,c[4]])

        fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="color", offset=12, datatype=PointField.UINT32, count=1),
        PointField(name="score", offset=16, datatype=PointField.FLOAT32, count=1)
        ]

        colored_map_msg = point_cloud2.create_cloud(header=msg.header, fields=fields,points=colored_map)
        colored_map_msg.is_dense = True
        
        self.colored_map_pub.publish(colored_map_msg)


        return colored_map


       



