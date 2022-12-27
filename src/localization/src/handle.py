import rospy
import tf
import tf2_ros
import geometry_msgs.msg

from sbg_driver.msg import SbgGpsPos, SbgEkfNav, SbgImuData, SbgGpsVel
from geometry_msgs.msg import Vector3



class Localization():


    def __init__(self):
        rospy.logwarn("Funcionando...")

        self.subscribe_topics()
        self.pub = None
        self.publish_topics()

    def subscribe_topics(self):
        rospy.Subscriber("/sbg/ekf_nav", SbgEkfNav, self.send_velocity)
        rospy.Subscriber("/sbg/gps_pos", SbgGpsPos, self.send_position)
        rospy.Subscriber("/sbg/imu_data", SbgImuData, self.send_acceleration)
        rospy.Subscriber("/sbg/gps_vel", SbgGpsVel, self.send_gps_velocity)


    def publish_topics(self):
        self.pub_vel = rospy.Publisher("/velocity", Vector3, queue_size=1)
        self.pub_pos = rospy.Publisher("/gps_position", Vector3, queue_size=1)
        self.pub_acc = rospy.Publisher("/acceleration", Vector3, queue_size=1)
        self.pub_gps_vel = rospy.Publisher("/gps_velocity", Vector3, queue_size=1)


    def send_velocity(self, msg):
        vel = Vector3()

        vel.x = msg.velocity.x 
        vel.y = msg.velocity.y
        vel.z = msg.velocity.z

        self.pub_vel.publish(vel)

    def send_gps_velocity(self, msg):
        gps_vel = Vector3()

        gps_vel.x = msg.velocity.x # North
        gps_vel.y = msg.velocity.y # East
        gps_vel.z = msg.velocity.z # Down

        self.pub_gps_vel.publish(gps_vel)
        
    def send_position(self, msg):
        position = Vector3()
        position.x = msg.latitude
        position.y = msg.longitude

        self.pub_pos.publish(position)

    def send_acceleration(self, msg):
        acc = Vector3()
        acc.x = msg.accel.x
        acc.y = msg.accel.y
        acc.z = msg.accel.z
    
        self.pub_acc.publish(acc)

    # TF Implementation. WIP.
    def tf_car_position(msg):
        br = tf.TransformBroadcaster()
        br.sendTransform(msg, rospy.Time.now(), "Position")