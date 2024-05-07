import rospy
from common_msgs.msg import Controls
from eposhandle import EPOSHandle
from std_msgs.msg import Float32MultiArray


MAX_ACCELERATION = rospy.get_param('~MAX_ACCELERATION', 1000)
MAX_DECELERATION = rospy.get_param('~MAX_DECELERATION', 1000)
PROFILE_VELOCITY = rospy.get_param('~PROFILE_VELOCITY', 1000)

class SteeringHandle:
    def __init__(self):
        self.epos = EPOSHandle(MAX_ACCELERATION, MAX_DECELERATION, PROFILE_VELOCITY)
        self.epos.connect_to_device()
        self.epos.enable()
        self._is_shutdown = False

        rospy.Subscriber('/controls', Controls,
                         callback=self.command_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1/20), self.epos_info_callback)
        self.info_pub = rospy.Publisher('/steering/epos_info', Float32MultiArray, queue_size=10)


    def command_callback(self, msg: Controls):
        assert msg.steering < 20 and msg.steering > -20
        if not self._is_shutdown:
            self.epos.move_to(msg.steering)

    def epos_info_callback(self,event):
        info = self.epos.get_epos_info()

        msg = Float32MultiArray()
        msg.data = info
        self.info_pub.publish(msg)

    def clean_and_close(self):
        self._is_shutdown = True
        self.epos.disable()
        self.epos.disconnect_device()
