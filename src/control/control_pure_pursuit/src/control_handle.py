"""Script to execute control node

@author: Mariano del Río
@date: 20220704
"""

import rospy
import tf2_ros
import tf_conversions
import time

from common_msgs.msg import Controls, Trajectory, CarState
from geometry_msgs.msg import Point
from common_msgs.srv import Spawn
from control import ControlCar
from fssim_common.msg import State
from std_msgs.msg import Float32
import math

# Constants
sim_mode = rospy.get_param('/control_pure_pursuit/simulation') # look for simulation mode

class ControlHandle():
    """Listen route and state and generate command controls of steering and
    accelerator.
    """

    def __init__(self):

        self.control = ControlCar(time.time())

        self.subscribe_topics()
        self.pub = None
        self.pub2 = None
        self.publish_topics()

        # self.tfBuffer = tf2_ros.Buffer()
        # tf2_ros.TransformListener(self.tfBuffer)

        # rospy.wait_for_service('spawn')
        # spawner = rospy.ServiceProxy('spawn', Spawn)
        # name = 'local_map'  # Name of frame
        # spawner(4, 2, 0, name)


    def subscribe_topics(self):

        topic1 = rospy.get_param('/control_pure_pursuit/route_topic')
        rospy.Subscriber(topic1, Trajectory, self.update_trajectory_callback)
        if sim_mode:
            rospy.Subscriber('/state', CarState, self.update_state_sim, queue_size=1)
        else:
            rospy.Subscriber('/motor_speed', Float32, self.update_state, queue_size=1)


    def publish_topics(self):

        topic_pub = rospy.get_param('/control_pure_pursuit/controls_topic')
        self.pub = rospy.Publisher(topic_pub, Controls, queue_size=1)

        topic_pursuit = rospy.get_param('/control_pure_pursuit/pursuit_topic')
        self.pub2 = rospy.Publisher(topic_pursuit, Point, queue_size=10)


    def update_state(self, msg: Float32):

        v = msg.data
        
        self.control.update_state(rospy.Time.now(), v)

    def update_state_sim(self, msg: CarState):

        v = math.hypot(msg.vx,msg.vy)  # get velocity from fssim
        
        self.control.update_state(rospy.Time.now(), v)


    def update_trajectory_callback(self, msg):

        data = msg.trajectory  # List of points

        pointsx = [p.x for p in data]
        pointsy = [p.y for p in data]

        self.control.update_trajectory(pointsx, pointsy)
        self.publish_msg()


    def publish_msg(self):

        ai, di = self.control.get_cmd()
        msg = Controls()
        msg.steering = di
        msg.accelerator = ai

        p = Point()
        ind = self.control.target_ind
        if(len(self.control.target_course.cx)>0):
            p.x = self.control.target_course.cx[ind]
            p.y = self.control.target_course.cy[ind]
            self.pub2.publish(p)

        self.pub.publish(msg)
