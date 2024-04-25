"""
@Author: Rafael Guil Valero
@Description: Script to execute FSDS interface node.
@Date: 2024-04-23
@LastUpdate: 2024-04-23
"""

from common_msgs.msg import CarState, Controls
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import rospy
from sensor_msgs.msg import PointCloud2


class transformerFSDS():
    """
    Listen topics from FSDS and transform them to
    our topics and vice versa.
    """

    def __init__(self):

        self.subscribe_topics()
        self.pub_state = None
        self.pub_cmd = None
        self.pub_pcl = None
        self.MAX_ANGLE = 25

        self.publish_topics()

    def publish_topics(self):
        self.pub_state = rospy.Publisher('/state', CarState, queue_size=10)
        self.pub_cmd = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=10)

    def subscribe_topics(self):
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.send_state)
        rospy.Subscriber('/controls', Controls, self.send_controllers)

    def send_state(self, msg):
        state = CarState()

        #Posicion del coche
        state.x = msg.pose.pose.position.x
        state.y = msg.pose.pose.position.y
        state.z = msg.pose.pose.position.z

        #Roll, pitch, yaw
        orientation_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)

        state.yaw = yaw
        state.roll = roll
        state.pitch = pitch

        #Velocidades lineales
        state.vx = msg.twist.twist.linear.x
        state.vy = msg.twist.twist.linear.y

        #Velocidad angular
        state.r = msg.twist.twist.angular.z

        rospy.loginfo(state)
        self.pub_state.publish(state)

    def send_controllers(self, msg):
        cmd = ControlCommand()
        cmd.throttle = msg.accelerator  # Aceleración
        cmd.steering = msg.steering / - self.MAX_ANGLE  # Dirección

        rospy.loginfo(cmd)
        self.pub_cmd.publish(cmd)