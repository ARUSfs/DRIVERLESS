"""Script to execute control node

@author: Mariano del Río
@date: 20220704
"""

import rospy
import tf2_ros
import tf_conversions

from common_msgs.msg import Controls, Trajectory
from geometry_msgs.msg import Point
from common_msgs.srv import Spawn
from control import ControlCar


class ControlHandle():
    """Listen route and state and generate command controls of steering and
    accelerator.
    """

    def __init__(self):

        time = rospy.Time.now()
        self.control = ControlCar(time)

        self.subscribe_topics()
        self.pub = None
        self.pub2 = None
        self.publish_topics()

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', Spawn)
        name = 'local_map'  # Name of frame
        spawner(4, 2, 0, name)

    def subscribe_topics(self):

        topic1 = rospy.get_param('/control_pure_pursuit/route_topic')
        rospy.Subscriber(topic1, Trajectory, self.update_trajectory_callback)

    def publish_topics(self):

        topic_pub = rospy.get_param('/control_pure_pursuit/controls_topic')
        self.pub = rospy.Publisher(topic_pub, Controls, queue_size=10)

        topic_pursuit = rospy.get_param('/control_pure_pursuit/pursuit_topic')
        self.pub2 = rospy.Publisher(topic_pursuit, Point, queue_size=10)

    def update_state(self):

        origin_frame = 'local_map'
        target_frame = 'global_map'
        trans = self.tfBuffer.lookup_transform(origin_frame,
                                               target_frame,
                                               rospy.Time.now(),
                                               rospy.Duration(1.0))

        # Read system reference data from TF
        x = trans.transform.translation.x
        y = trans.transform.translation.y

        q = (trans.transform.rotation.x,
             trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w)
        euler_angles = tf_conversions.transformations.euler_from_quaternion(q)
        yaw = euler_angles[2]

        v = 0  # Not using velocity now.

        self.control.update_state(rospy.Time.now(), x, y, yaw, v)

    def update_trajectory_callback(self, msg):

        data = msg.trajectory  # List of points

        pointsx = [p.x for p in data]
        pointsy = [p.y for p in data]

        self.control.update_trajectory(pointsx, pointsy)
        self.publish_msg()

    def publish_msg(self):

        self.update_state()

        ai, di = self.control.get_cmd()
        msg = Controls()
        msg.steering = di
        msg.accelerator = ai

        p = Point()
        ind = self.control.target_ind
        p.x = self.control.target_course.cx[ind]
        p.y = self.control.target_course.cy[ind]
        self.pub2.publish(p)

        rospy.loginfo(msg)
        self.pub.publish(msg)
