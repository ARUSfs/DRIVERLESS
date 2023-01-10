import rospy
import tf2_ros
import geometry_msgs.msg

from sbg_driver.msg import SbgGpsPos, SbgEkfNav, SbgImuData, SbgGpsVel
from geometry_msgs.msg import Vector3



class Localization():

    def __init__(self):
        rospy.logwarn("Running Localization Node...")

        self.subscribe_topics()
        self.pub = None
        self.publish_topics()

    def subscribe_topics(self):
        rospy.Subscriber("/sbg/ekf_nav", SbgEkfNav, self.send_velocity)
        rospy.Subscriber("/sbg/gps_pos", SbgGpsPos, self.send_position)
        rospy.Subscriber("/sbg/imu_data", SbgImuData, self.send_acceleration)
        rospy.Subscriber("/sbg/gps_vel", SbgGpsVel, self.send_gps_velocity)
        rospy.Subscriber("/gps_position", Vector3, self.tf2_start_point)
        rospy.Subscriber("/gps_position", Vector3, self.tf2_imu_position)
        rospy.Subscriber("/gps_position", Vector3, self.tf2_camera_position)

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

    # TF2 IMPLEMENTATION
    def tf2_start_point(self, msg):
        br = tf2_ros.StaticTransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "start"
        
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)

    def tf2_imu_position(self, msg):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "start"
        t.child_frame_id = "imu"
        
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)

    def tf2_camera_position(self, msg):
        br = tf2_ros.StaticTransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "imu"
        t.child_frame_id = "camera"
        
        t.transform.translation.x = 0.0 # Modificar para que sea la distancia entre el IMU y la camara
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)