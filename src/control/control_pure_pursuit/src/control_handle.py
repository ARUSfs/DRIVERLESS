"""
Script to execute control node

@author: Mariano del Río
@date: 20220704
"""

import rospy
import math

from common_msgs.msg import Controls, Trajectory, CarState
from control import controlCar


class controlHandle():
    """
    Listen route and state and generate command controls of steering and
    accelerator.
    """

    def __init__(self):

        time = rospy.Time.now()
        self.control = controlCar(time)

        self.subscribe_topics()
        self.pub = None
        self.publish_topics()

    def subscribe_topics(self):

        topic1 = rospy.get_param('/control_pure_pursuit/route_topic')
        rospy.Subscriber(topic1, Trajectory, self.update_trajectory_callback)

        topic2 = rospy.get_param('/control_pure_pursuit/state_topic')
        rospy.Subscriber(topic2, CarState, self.update_state_callback)

    def publish_topics(self):

        topic_pub = rospy.get_param('/control_pure_pursuit/controls_topic')
        self.pub = rospy.Publisher(topic_pub, Controls, queue_size=10)

    def update_state_callback(self, msg):

        x = msg.x.data
        y = msg.y.data
        yaw = msg.yaw.data
        v = math.sqrt((msg.vx.data**2 + msg.vy.data**2))

        self.control.update_state(rospy.Time.now(), x, y, yaw, v)

    def update_trajectory_callback(self, msg):

        data = msg.trajectory  # List of points

        pointsx = [p.x for p in data]
        pointsy = [p.y for p in data]

        self.control.update_trajectory(pointsx, pointsy)

    def run(self):

        if self.control.existPath:
            self.publish_msg()

        else:
            rospy.loginfo("Not route yet")

    def publish_msg(self):

        ai, di = self.control.get_cmd()
        msg = Controls()
        msg.steering.data = di
        msg.accelerator.data = ai

        rospy.loginfo(msg)
        self.pub.publish(msg)
