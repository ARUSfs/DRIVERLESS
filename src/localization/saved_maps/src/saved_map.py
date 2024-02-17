'''
@Author: Carlos Pérez Cantalapiedra
@Description: Make a list of maps and publish them.
@Date: 2024-02-15
'''

import rospy
from common_msgs.msg import Map, MapList
from visualization_msgs.msg import MarkerArray, Marker

class SavedMap:
    def __init__(self):
        self.map = Map()
        self.mapList = MapList()
        self.map_sub = rospy.Subscriber('/map', Map, self.map_callback, queue_size=1)
        self.pub = rospy.Publisher('/map_list', MapList, queue_size=1)
        self.pmarkers = rospy.Publisher('/final_map', MarkerArray, queue_size=1)

    
    def map_callback(self, msg):
        self.map = msg
        self.mapList.map_list.append(msg)
        self.pub.publish(self.mapList)
        self.pub_markers(self.map)


    def pub_markers(self, map):
        marray = MarkerArray()
        m1 = Marker()
        m1.id = 100000
        m1.action = Marker.DELETEALL
        marray.markers.append(m1)
        for i, lm in enumerate(map.cones):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time().now()
            marker.type = Marker.CYLINDER
            marker.action = Marker.MODIFY
            marker.pose.position.x = lm.position.x
            marker.pose.position.y = lm.position.y
            marker.pose.position.z = lm.position.z
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            if lm.color == 'b':
                marker.color.r = 0
                marker.color.g = 0
                marker.color.b = 1
                marker.color.a = 1.0
            elif lm.color == 'y':
                marker.color.r = 1
                marker.color.g = 1
                marker.color.b = 0
                marker.color.a = 1.0
            else:
                marker.color.r = 0
                marker.color.g = 0
                marker.color.b = 0
                marker.color.a = 1.0

            marker.lifetime = rospy.Duration()
            marray.markers.append(marker)
            
        self.pmarkers.publish(marray)