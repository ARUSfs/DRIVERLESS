"""
Script to control the car with a Dualshock4

@author: Rafael Guil Valero
@date: 15/05/24
"""

import rospy
from ds4_driver.msg import Status
from common_msgs.msg import Controls

CMD = rospy.get_param('/control_joystick/CMD')
DELTA = rospy.get_param('/control_joystick/DELTA')

class joystick():
    def __init__(self):

        self.subscribe_topics()
        self.pub = None
        self.publish_topics()

    def publish_topics(self):
        self.pub = rospy.Publisher('/controls', Controls, queue_size=10)
    
    def subscribe_topics(self):
        rospy.Subscriber('/status', Status, self.send_controllers)

    def send_controllers(self, msg: Status) :
        cmd = Controls()
        cmd.accelerator = msg.axis_left_y * CMD
        cmd.steering = msg.axis_right_x * DELTA
        self.pub.publish(cmd)