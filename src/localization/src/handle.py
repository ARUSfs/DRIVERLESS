import rospy

from sbg_driver.msg import SbgGpsPos, SbgEkfEuler, SbgImuData, SbgGpsVel
from sbg_driver.msg import SbgMag
from sbg_driver.msg import SbgEkfNav
from common_msgs.msg import velState



class localization():


    def init(self):


        self.subscribe_topics()
        self.pub = None
        self.publish_topics()

    def subscribe_topics(self):
        rospy.Subscriber("/sbg/ekf_nav", SbgEkfNav, self.send_velocity)


    def publish_topics(self):
        self.pub = rospy.Publisher("vel_state", velState, queue_size=1)
        

    def send_velocity(self, msg):
        vel = velState()
        vel.vx = msg.velocity.x
        vel.vy = msg.velocity.y
        vel.vz = msg.velocity.z

        self.pub.publish(vel)
        

