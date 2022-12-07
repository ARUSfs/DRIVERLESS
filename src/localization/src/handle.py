import rospy
import tf
import tf2_ros
import geometry_msgs.msg

from sbg_driver.msg import SbgGpsPos, SbgEkfEuler, SbgImuData, SbgGpsVel, SbgMag, SbgEkfNav
from common_msgs.msg import velState



class Localization():


    def init(self):


        self.subscribe_topics()
        self.pub = None
        self.publish_topics()

    def subscribe_topics(self):
        rospy.Subscriber("/sbg/ekf_nav", SbgEkfNav, self.send_velocity)
        rospy.Subscriber("/sbg/gps_pos", SbgGpsPos, self.send_position)
        rospy.Subscriber("/sbg/mag", SbgMag, self.send_acceleration)


    def publish_topics(self):
        self.pub = rospy.Publisher("/vel_state", velState, queue_size=1)
        self.pub = rospy.Publisher("/gps_position", SbgGpsPos, queue_size=1)
        self.pub = rospy.Publisher("/acceleration", SbgMag, queue_size=1)


    def send_velocity(self, msg):
        vel = velState()
        vel.vx = msg.velocity.x
        vel.vy = msg.velocity.y
        vel.vz = msg.velocity.z

        self.pub.publish(vel)
        
    def send_position(self, msg):
        position = SbgGpsPos()
        position.latitude = msg.latitude
        position.longitude = msg.longitude

        self.pub.publish(position)

    def send_acceleration(self, msg):
        mag = SbgMag()
        mag.accel = msg.acceleration

        self.pub.publish(mag)