import rospy
from geometry_msgs.msg import Point
from common_msgs.msg import Trajectory, CarState
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.interpolate import splprep, splev
import time
import math


class GlobalRouteHandler():

    def __init__(self):

        self.i = 0
        self.N = 1000

        self.FIRST_LAP = True
        self.t_first_lap = 0

        self.sk_pub = rospy.Publisher('/controller/sk',Trajectory,queue_size=1)
        self.i_phi_dist_msg = rospy.Publisher('/i_phi_dist', Float32MultiArray, queue_size=1)
        self.pub_curva = rospy.Publisher('/mpc/curva', MarkerArray, queue_size=1)

        rospy.Subscriber('/delaunay/global_route',Trajectory,self.set_mpc_route,queue_size=1)
        rospy.Subscriber('/car_state/state', CarState, self.update_state, queue_size=1)

    

    def update_state(self, msg:CarState):
        if (not self.FIRST_LAP):

            rot = np.array([[math.cos(-msg.yaw),-math.sin(-msg.yaw)],[math.sin(-msg.yaw),math.cos(-msg.yaw)]])
            local_route = (self.route-np.array([msg.x,msg.y])) @ rot.T

            if self.i+self.N/10 < len(local_route):
                # se halla el punto más cercano al eje delantero
                dist = np.linalg.norm(local_route[self.i:self.i+int(self.N/10),:]-np.array([0.94,0]),axis=1)
                self.i+=np.argmin(dist)
                self.i = self.i%self.N
            else:
                dist1 = np.linalg.norm(local_route[self.i:,:]-np.array([0.94,0]),axis=1)
                dist2 = np.linalg.norm(local_route[:int(self.N/10)-self.i,:]-np.array([0.94,0]),axis=1)
                dist = np.hstack([dist1,dist2])
                self.i+=np.argmin(dist)
                self.i = self.i%self.N
            signo_d = -np.sign(local_route[self.i,1])
            
            d_min=np.min(dist)*signo_d
            dist = d_min+0.0001

            phi = -np.arctan2(local_route[self.i+1,1]-local_route[self.i,1],local_route[self.i+1,0]-local_route[self.i,0])

            i_phi_dist_msg = Float32MultiArray()
            i_phi_dist_msg.data.append(self.i)
            i_phi_dist_msg.data.append(phi)
            i_phi_dist_msg.data.append(dist)
            self.i_phi_dist_msg.publish(i_phi_dist_msg)


     

    def set_mpc_route(self, msg:Trajectory):
        if self.FIRST_LAP:

            r = np.array([[msg.trajectory[i].x,msg.trajectory[i].y] for i in range(0,len(msg.trajectory),2)])
            
            tck, u = splprep(r.T, s=3, per=True)  
            u_new = np.linspace(u.min(), u.max(), self.N)
            puntos_curva = np.array(splev(u_new, tck))

            self.route = puntos_curva.T

            acum=0
            s=[]
            s.append(0)
            xp = []
            yp = []
            for i in range(len(puntos_curva[0])-1):
                p1=puntos_curva[:,i]
                p2=puntos_curva[:,i+1]
                xp.append(p2[0]-p1[0])
                yp.append(p2[1]-p1[1])
                acum+=np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
                s.append(acum)
            xp.append(xp[-1])
            yp.append(yp[-1])


            xpp=[]
            ypp=[]
            for i in range(len(xp)-1):
                xpp.append(xp[i+1]-xp[i])
                ypp.append(yp[i+1]-yp[i])
            xpp.append(xpp[-1])
            ypp.append(xpp[-1])

            k=[]
            for i in range(len(xpp)):
                if xp[i]!=yp[i]:
                    k.append((xp[i]*ypp[i] - xpp[i]*yp[i])/(xp[i]**2+yp[i]**2)**1.5)
                else:
                    k.append(0)
            
            

            sk_msg = Trajectory()
            sk_msg.trajectory = [Point(s[i],k[i],0) for i in range(len(s))]

            # self.subscribe_topics()
            self.FIRST_LAP=False
            self.t_first_lap = time.time()
            
            self.sk_pub.publish(sk_msg)

            curvas = MarkerArray()
            curva = Marker()
            curva.header.frame_id = "map"
            curva.ns = 'mpc'
            curva.id = 0
            curva.action = Marker.ADD
            curva.type = Marker.LINE_STRIP
            curva.points = [Point(p[0],p[1],0) for p in puntos_curva.T]
            curva.color.r = 1
            curva.color.g = 0.5
            curva.color.a = 1
            curva.scale.x = 0.1
            curva.pose.orientation.x = 0.0
            curva.pose.orientation.y = 0.0
            curva.pose.orientation.z = 0.0
            curva.pose.orientation.w = 1.0
            curvas.markers.append(curva)

            self.pub_curva.publish(curvas)

            
