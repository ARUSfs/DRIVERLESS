"""Script to execute planning node

@author: Mariano del Río
@date: 20220504
"""

from itertools import combinations
from common_msgs.msg import Trajectory, CarState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from common_msgs.msg import Simplex, Triangulation
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from scipy.interpolate import splprep, splev
from std_msgs.msg import Int16, Float32MultiArray
import numpy as np
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs

import rospy

from planning import PlanningSystem
from track_planning import TrackPlanningSystem

global_frame = rospy.get_param('/delaunay_detector/global_frame')
car_frame = rospy.get_param('/delaunay_detector/car_frame')
slam = rospy.get_param('/delaunay_detector/slam')
TRACKDRIVE = rospy.get_param('/delaunay_detector/trackdrive')
SMOOTH = rospy.get_param('/delaunay_detector/SMOOTH')

class PlanningHandle():
    """Listen map of cones, calculate center track via Delaunay
    triangulation and publish trajectory message.
    Every message received of cones, a new trajectory is published.
    """

    def __init__(self):
        self.first_lap = True

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

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

        if(SMOOTH):
            rospy.Subscriber("/car_state/state", CarState, self.update_state)
            self.pub_speed_profile = rospy.Publisher("/speed_profile", Float32MultiArray, queue_size=1)
            self.pub_sk = rospy.Publisher('/controller/sk',Trajectory,queue_size=1)


    def get_trajectory(self, msg):
        if(slam=="marrano"):
            # usamos el mapeado global como percepción local para ganar estabilidad
            trans=self.tf_buffer.lookup_transform(car_frame, global_frame, rospy.Time(0))
            trans_msg = tf2_sensor_msgs.do_transform_cloud(msg,transform=trans)
            cones_all = point_cloud2.read_points(trans_msg, field_names=("x", "y", "z","color", "score"), skip_nans=True)
            cones = [c for c in cones_all if c[0]>-5 and c[0]<15 and np.abs(c[1])<7]
        else:
            cones = point_cloud2.read_points(msg, field_names=("x", "y", "z","color","score"),skip_nans=True)
        
        if (self.first_lap or not TRACKDRIVE):
            self.planning_system.update_tracklimits(cones)
        self.publish_msg()


    def publish_msg(self):
        self.planning_system.calculate_path()
        self.delaunay_publisher.publish(self.planning_system.triang)

        msg = Trajectory()
        msg.trajectory = list()
        for midpoint in self.planning_system.route:
            point = Point()
            point.x = midpoint[0]
            point.y = midpoint[1]
            point.z = 0
            msg.trajectory.append(point)

        self.pub_route.publish(msg)

        if(SMOOTH and len(self.planning_system.speed_profile)>0):
            speed_profile_msg = Float32MultiArray()
            speed_profile_msg.data=self.planning_system.speed_profile
            self.pub_speed_profile.publish(speed_profile_msg)

            s = self.planning_system.s
            k = self.planning_system.k

            sk_msg = Trajectory()
            msg.trajectory = list()
            for i in range(len(s)):
                p = Point()
                p.x = s[i]
                p.y = k[i]
                p.z = 0
                sk_msg.trajectory.append(p)
            self.pub_sk.publish(sk_msg)


    def get_global_track(self,msg :PointCloud2):
        if self.first_lap:
            self.first_lap = False

            cones = point_cloud2.read_points(msg, field_names=("x", "y", "z","color", "score"), skip_nans=True)
            self.planning_system.update_tracklimits(cones)
            self.planning_system.calculate_path()

            
            # rospy.logwarn(route)
            msg2 = Trajectory()
            msg2.trajectory = [Point(p[0],p[1],0) for p in self.planning_system.route[1:]]
            # rospy.logwarn(msg2)
            rospy.logerr(len(msg2.trajectory))

            self.pub_global_route.publish(msg2)

    def update_state(self, msg: CarState):
        self.planning_system.speed = np.hypot(msg.vx,msg.vy)



