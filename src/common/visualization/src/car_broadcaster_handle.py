"""Script to add car reference system to tree tf

@author: Mariano del Río
@date: 20220804
"""

from common_msgs.msg import CarState
import geometry_msgs.msg

import rospy
import tf2_ros
import tf_conversions


class CarBroadcasterHandle():

    def __init__(self):

        self.br = tf2_ros.TransformBroadcaster()
        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = "global_map"
        self.t.child_frame_id = "local_map"

        self.subscribeToTopics()

    def subscribeToTopics(self):
        topic_state = rospy.get_param("/interface/topic_state")
        rospy.Subscriber(topic_state, CarState, self.transformState)

    def transformState(self, msg):

        # Position is zero due to local system
        self.t.header.stamp = rospy.Time.now()
        self.t.transform.translation.x = 0
        self.t.transform.translation.y = 0
        self.t.transform.translation.z = 0

        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, msg.yaw)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]

        self.br.sendTransform(self.t)  # Send transformation
