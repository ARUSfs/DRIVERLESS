import rospy
from std_msgs.msg import Float32
from common_msgs.msg import CarState

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
        self.subscribe_topics()
        self.publish_topics()
        
        # Timer to periodically publish state
        rospy.Timer(rospy.Duration(0.01), self.publish_state)

    def motorspeed_callback(self, msg):
        self.vx = msg.data

    def subscribe_topics(self):
        rospy.Subscriber('/motor_speed', Float32, self.motorspeed_callback)

    def publish_topics(self):
        self.pub_state = rospy.Publisher('/state/car_state', CarState, queue_size=10)
     
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