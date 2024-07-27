import rospy
from common_msgs.msg import Controls, CarState
from eposhandle import EPOSHandle
from std_msgs.msg import Float32MultiArray,Float32
import math


MAX_ACCELERATION = rospy.get_param('/steering/MAX_ACCELERATION',default=6000)
MAX_DECELERATION = rospy.get_param('/steering/MAX_DECELERATION',default=6000)
PROFILE_VELOCITY = rospy.get_param('/steering/PROFILE_VELOCITY',default=6000)
holgura = 0.6
L = 1.535
CORRIGE_HOLGURA = False

class SteeringHandle:
    def __init__(self):
        self.speed = 0
        self.r = 0
        self.epos = EPOSHandle(MAX_ACCELERATION, MAX_DECELERATION, PROFILE_VELOCITY)
        self.epos.connect_to_device()
        self.epos.enable()

        # self.initial_position = rospy.wait_for_message("/can/extensometer",Float32)
        # self.epos.set_position_offset(self.initial_position.data)

        self._is_shutdown = False

        rospy.Subscriber('/controls', Controls, callback=self.command_callback, queue_size=1)
        rospy.Subscriber('/car_state/state', CarState, callback=self.update_state)
        self.info_pub = rospy.Publisher('/steering/epos_info', Float32MultiArray, queue_size=10)


    def command_callback(self, msg: Controls):
        assert msg.steering <= 20 and msg.steering >= -20
        if not self._is_shutdown:
            if CORRIGE_HOLGURA and self.speed > 2:
                k_cmd = msg.steering/L
                k_actual = self.r/self.speed
                if k_cmd>k_actual:
                    self.epos.move_to(-(msg.steering+holgura))
                else:
                    self.epos.move_to(-(msg.steering-holgura))
            else:
                self.epos.move_to(-msg.steering)

        epos_info = self.epos.get_epos_info()
        msg = Float32MultiArray()
        msg.data = epos_info
        self.info_pub.publish(msg)

    def clean_and_close(self):
        self._is_shutdown = True
        self.epos.disable()
        self.epos.disconnect_device()

    def update_state(self, msg: CarState):
        self.speed = math.hypot(msg.vx,msg.vy)
        self.r = msg.r
