import rospy
from std_msgs.msg import Float32
from common_msgs.msg import CarState
import tf2_ros
import tf.transformations
from geometry_msgs.msg import TransformStamped
from fssim_common.msg import State

global_frame = rospy.get_param('/car_state/global_frame')
car_frame = rospy.get_param('/car_state/car_frame')
get_base_pose_position = rospy.get_param('/car_state/get_base_pose_position')
SLAM = rospy.get_param('/car_state/SLAM')

class StateClass:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0 
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.r = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.pub_state = None
        
        # Initialize subscribers and publishers
        self.pub_state = rospy.Publisher('/car_state/state', CarState, queue_size=10)
        rospy.Subscriber('/motor_speed', Float32, self.motorspeed_callback)
        rospy.Subscriber('fssim/base_pose_ground_truth', State, self.base_pose_callback)
        
        # Timer to periodically update global position
        if SLAM != "none":
            self.tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            rospy.Timer(rospy.Duration(0.01), self.update_position)
        # Timer to periodically publish state
        rospy.Timer(rospy.Duration(0.01), self.publish_state)

    def motorspeed_callback(self, msg):
        self.vx = msg.data
    
    def base_pose_callback(self,msg: State):
        self.vx = msg.vx
        self.vy = msg.vy
        if get_base_pose_position:
            self.x = msg.x
            self.y = msg.y
            self.yaw = msg.yaw
            self.r = msg.r


    def update_position(self, event):

        try:
            transform = self.tf_buffer.lookup_transform(global_frame, car_frame, rospy.Time(0), rospy.Duration(1.0))
            orientation = transform.transform.rotation
            euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            if SLAM == "marrano":
                self.x = transform.transform.translation.x
                self.y = transform.transform.translation.y
                self.z = transform.transform.translation.z
                self.yaw = euler[2]
            elif SLAM == "limovelo":
                self.x = transform.transform.translation.x
                self.y = -transform.transform.translation.y
                self.z = -transform.transform.translation.z
                self.yaw = euler[2]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform not available")
     
    def publish_state(self, event):
        state = CarState()
        state.vx = self.vx
        state.vy = self.vy
        state.vz = self.vz
        state.r = self.r
        state.x = self.x
        state.y = self.y
        state.z = self.z
        state.roll = self.roll
        state.pitch = self.pitch
        state.yaw = self.yaw
        self.pub_state.publish(state)