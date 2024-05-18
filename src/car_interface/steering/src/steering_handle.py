import rospy
from common_msgs.msg import Controls
from eposhandle import EPOSHandle
from std_msgs.msg import Float32MultiArray,Float32


MAX_ACCELERATION = 6000
MAX_DECELERATION = 6000
PROFILE_VELOCITY = 1500

class SteeringHandle:
    def __init__(self):
        self.epos = EPOSHandle(MAX_ACCELERATION, MAX_DECELERATION, PROFILE_VELOCITY)
        self.epos.connect_to_device()
        self.epos.enable()

        # self.initial_position = rospy.wait_for_message("/can/extensometer",Float32)
        # self.epos.set_position_offset(self.initial_position.data)

        self._is_shutdown = False

        rospy.Subscriber('/controls', Controls, callback=self.command_callback, queue_size=1)
        self.info_pub = rospy.Publisher('/steering/epos_info', Float32MultiArray, queue_size=10)


    def command_callback(self, msg: Controls):
        assert msg.steering < 20 and msg.steering > -20
        if not self._is_shutdown:
            self.epos.move_to(msg.steering)

        epos_info = self.epos.get_epos_info()
        msg = Float32MultiArray()
        msg.data = epos_info
        self.info_pub.publish(msg)

    def clean_and_close(self):
        self._is_shutdown = True
        self.epos.disable()
        self.epos.disconnect_device()
