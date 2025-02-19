import rospy
from std_msgs.msg import Float32, Float32MultiArray, String
from common_msgs.msg import CarState
import tf2_ros
import tf.transformations
from fssim_common.msg import State
from sensor_msgs.msg import Imu
import numpy as np
import math
import subprocess
import rospkg
import os

global_frame = rospy.get_param('/car_state/global_frame')
car_frame = rospy.get_param('/car_state/car_frame')
get_base_pose_position = rospy.get_param('/car_state/get_base_pose_position')
SLAM = rospy.get_param('/car_state/SLAM')
V_ESTIMATION = rospy.get_param('/car_state/V_ESTIMATION')
N_KMEANS = rospy.get_param('/car_state/N_KMEANS')

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

        PATH = rospkg.RosPack().get_path('can_c')

        self.yaw_ini = None

        self.limovelo_rotation = np.array([[1,0,0],[0,1,0],[0,0,1]])
        
        os.chdir(self.extract_workspace_path(PATH))
        self.commit_hash = subprocess.check_output(['git', 'rev-parse', 'HEAD']).strip().decode('utf-8')
        
        # Initialize subscribers and publishers
        self.pub_state = rospy.Publisher('/car_state/state', CarState, queue_size=10)
        self.pub_commit_hash = rospy.Publisher('/car_state/commit_hash', String, queue_size=10)
        rospy.Subscriber('/motor_speed', Float32, self.motorspeed_callback)
        rospy.Subscriber('fssim/base_pose_ground_truth', State, self.base_pose_callback)
        rospy.Subscriber('/can/IMU', Imu, self.imu_Callback)
        
        # Timer to periodically update global position
        if SLAM != "none":
            self.tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            rospy.Timer(rospy.Duration(0.01), self.update_position)
            if SLAM == "limovelo":
                rospy.Subscriber('/limovelo/rotation', Float32MultiArray, self.update_limovelo_rotation)
                
        # Time to periodically publish state
        rospy.Timer(rospy.Duration(0.01), self.publish_state)
        rospy.Timer(rospy.Duration(0.1), self.commit_hash_callback)

    def extract_workspace_path(self, full_path):
        # Definir el patrón que identifica el final de la parte del workspace
        pattern = "/DRIVERLESS/"

        # Encontrar la posición del patrón en la cadena completa
        pos = full_path.find(pattern)
        if pos != -1:
            # Extraer la subcadena hasta el final del patrón
            return full_path[:pos + len(pattern) - 1]
        else:
            # Si el patrón no se encuentra, devolver la ruta completa
            return full_path

    def imu_Callback(self, msg: Imu):
         # Extrae el cuaternión del mensaje IMU
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        # Convierte el cuaternión a roll, pitch, yaw
        roll, pitch, yaw = self.quaternion_to_euler(q)
        if self.yaw_ini == None:
            self.yaw_ini = - yaw
        if SLAM == 'none':
            self.yaw = ((-yaw - self.yaw_ini)+np.pi)%(2*np.pi) - np.pi 
        self.r = - msg.angular_velocity.z

    def quaternion_to_euler(self, q):
        x, y, z, w = q

        # Conversión a ángulos de Euler (roll, pitch, yaw)
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        return roll, pitch, yaw

    def motorspeed_callback(self, msg):
        if V_ESTIMATION == 'N_MEANS':
            self.vx = msg.data*(1/N_KMEANS) + self.vx*(1-1/N_KMEANS)
        elif V_ESTIMATION == 'KALMANN_FILTER':
            self.vx = self.getKalmanEstimation(msg.data)
        else:
            self.vx = msg.data
    
    def base_pose_callback(self,msg: State):
        self.vx = msg.vx
        self.vy = msg.vy
        self.yaw = msg.yaw
        self.r = msg.r
        if get_base_pose_position:
            self.x = msg.x
            self.y = msg.y

    def update_limovelo_rotation(self, msg: Float32MultiArray):
        self.limovelo_rotation = np.array([[msg.data[0],msg.data[1],msg.data[2]],
                                           [msg.data[3],msg.data[4],msg.data[5]],
                                           [msg.data[6],msg.data[7],msg.data[8]]])


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
                trans = np.array([transform.transform.translation.x,
                                  transform.transform.translation.y,
                                  transform.transform.translation.z])
                pos = self.limovelo_rotation@trans
                self.x = pos[0]
                self.y = -pos[1]
                self.z = -pos[2]
                self.yaw = - euler[2]
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

    def getKalmanEstimation(self, speed):
        pass

    def commit_hash_callback(self, event):
        self.pub_commit_hash.publish(self.commit_hash)

