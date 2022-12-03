import rospy
import tf
import tf2_ros
import geometry_msgs.msg

from sbg_driver.msg import SbgGpsPos, SbgEkfEuler, SbgImuData, SbgGpsVel
from sbg_driver.msg import SbgMag
from sbg_driver.msg import SbgEkfNav
from common_msgs.msg import velState, GpsPos



class localization():


    def init(self):


        self.subscribe_topics()
        self.pub = None
        self.publish_topics()

    def subscribe_topics(self):
        rospy.Subscriber("/sbg/ekf_nav", SbgEkfNav, self.send_velocity)
        rospy.Subscriber("/sbg/gps_pos", SbgGpsPos, self.send_position)


    def publish_topics(self):
        self.pub = rospy.Publisher("vel_state", velState, queue_size=1)
        self.pub = rospy.Publisher("gps position", GpsPos, queue_size=1)


    def send_velocity(self, msg):
        vel = velState()
        vel.vx = msg.velocity.x
        vel.vy = msg.velocity.y
        vel.vz = msg.velocity.z

        self.pub.publish(vel)
        
    def send_position(self, msg):
        position = GpsPos()
        position.latitude = msg.latitude
        position.longitude = msg.longitude
        position.altitude = msg.altitude

        self.pub.publish(position)


        #br = tf2_ros.TransforBroadcaster()
        #t = geometry_msgs.msg.TransformStamped()

        #t.header.stamp = rospy.Time.now() # Marca de tiempo
        #t.header.frame_id = "position" #Nombre del marco padre
        #t.child_frame_id = position