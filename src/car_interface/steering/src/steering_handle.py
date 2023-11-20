import rospy
from common_msgs.msg import Controls
from eposhandle import EPOSHandle


MAX_ACCELERATION = rospy.get_param('~MAX_ACCELERATION', 1000)
MAX_DECELERATION = rospy.get_param('~MAX_DECELERATION', 1000)
PROFILE_VELOCITY = rospy.get_param('~PROFILE_VELOCITY', 1000)

class SteeringHandle:
    def __init__(self):
        self.epos = EPOSHandle(MAX_ACCELERATION, MAX_DECELERATION, PROFILE_VELOCITY)
        self.epos.connect_to_device()
        self.epos.enable()
        self._is_shutdown = False

        rospy.Subscriber('/control_pure_pursuit/controls_topic', Controls,
                         callback=self.command_callback, queue_size=1)


    def command_callback(self, msg: Controls):
        assert msg.steering < 20 and msg.steering > -20
        if not self._is_shutdown:
            self.epos.move_to(msg.steering)

    def clean_and_close(self):
        self._is_shutdown = True
        self.epos.disable()
        self.epos.disconnect_device()
