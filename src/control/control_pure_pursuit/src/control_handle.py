"""Script to execute control node

@author: Mariano del Río
@date: 20220704
"""

import rospy
import math

from common_msgs.msg import Controls, Trajectory, CarState
from geometry_msgs.msg import Point
from control import ControlCar


class ControlHandle():
    """Listen route and state and generate command controls of steering and
    accelerator.
    """

    def __init__(self):

        time = rospy.Time.now()
        self.control = ControlCar(time)

        self.subscribe_topics()
        self.pub = None
        self.pub2 = None
        self.publish_topics()

    def subscribe_topics(self):

        topic1 = rospy.get_param('/control_pure_pursuit/route_topic')
        rospy.Subscriber(topic1, Trajectory, self.update_trajectory_callback)

        topic2 = rospy.get_param('/control_pure_pursuit/state_topic')
        rospy.Subscriber(topic2, CarState, self.update_state_callback)

    def publish_topics(self):

        topic_pub = rospy.get_param('/control_pure_pursuit/controls_topic')
        self.pub = rospy.Publisher(topic_pub, Controls, queue_size=10)

        topic_pursuit = rospy.get_param('/control_pure_pursuit/pursuit_topic')
        self.pub2 = rospy.Publisher(topic_pursuit, Point, queue_size=10)

    def update_state_callback(self, msg):

        x = msg.x
        y = msg.y
        yaw = msg.yaw
        v = math.sqrt((msg.vx**2 + msg.vy**2))

        self.control.update_state(rospy.Time.now(), x, y, yaw, v)

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
        p.x = self.control.target_course.cx[ind]
        p.y = self.control.target_course.cy[ind]
        self.pub2.publish(p)

        rospy.loginfo(msg)
        self.pub.publish(msg)
