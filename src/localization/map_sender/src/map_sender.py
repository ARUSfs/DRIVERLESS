import rospy
from std_msgs.msg import Bool
from common_msgs.msg import MapState

class Map_sender:
    
    def __init__(self):
        
        rospy.Subscriber('/map', MapState, self.save_map, queue_size=1)
        rospy.Subscriber('/map_query', Bool,self.send_map, queue_size=1)
        self.send_map = rospy.Publisher('/map', MapState, queue_size=1)
        self.map = MapState()

    def save_map(self, msg: MapState):
        self.map = msg
        rospy.loginfo(self.map)

    def send_map(self, msg: Bool):
        if msg.data:
            self.send_map.publish(self.map)
            rospy.loginfo("Map sent")
            
        