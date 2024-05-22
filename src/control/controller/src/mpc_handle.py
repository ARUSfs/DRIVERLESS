import rospy
from geometry_msgs.msg import Point
from common_msgs.msg import Trajectory
from fssim_common.msg import State
from visualization_msgs.msg import Marker,MarkerArray
from common_msgs.srv import MPCCurrentState, MPCCurrentStateResponse
import numpy as np
from scipy.interpolate import splprep, splev
import time
import math


class MPCHandle():

    def __init__(self):

        self.si=0
        self.dist=0
        self.phi=0
        self.vx=0
        self.delta=0
        self.par=0

        self.FIRST_LAP = True
        self.t_first_lap = 0

        self.x = []
        self.y = []
        self.s = []

        self.sk_pub = rospy.Publisher('/controller/sk',Trajectory,queue_size=1)
        self.pub_curva = rospy.Publisher('/mpc/curva', MarkerArray, queue_size=1)

        rospy.Subscriber('/delaunay/global_route',Trajectory,self.set_mpc_route,queue_size=1)
        rospy.Subscriber('/fssim/base_pose_ground_truth', State, self.update_mpc_state, queue_size=1)

        rospy.Service('get_x_msg', MPCCurrentState, self.get_x_msg)

        


    def get_x_msg(self,req):
        m = MPCCurrentStateResponse(self.si,self.dist,self.phi,self.vx,self.par,self.delta)
        return m
    

    def update_mpc_state(self, msg:State):
        if (not self.FIRST_LAP):
            x = self.x
            y = self.y
            s = self.s

            dx=np.array(x)-msg.x
            dy=np.array(y)-msg.y
            dist = [np.linalg.norm([dx[i],dy[i]]) for i in range(len(x))]
            i=np.argmin(dist)

            if x[i]!=x[i+1]:
                recta=(y[i+1]-y[i])/(x[i+1]-x[i])*(msg.x-x[i])+y[i]-msg.y
                signo_d=np.sign(recta)*np.sign(x[i]-x[i+1])
            else:
                signo_d=np.sign(y[i+1]-y[i])*np.sign(msg.x-x[i])

            d_min=np.min(dist)*signo_d

            corrected_yaw = (msg.yaw+np.pi)%(2*np.pi) - np.pi
            theta = np.arctan2(y[(i+5)%len(x)]-y[i],x[(i+5)%len(x)]-x[i])
            phi = corrected_yaw - theta
            phi_corrected = corrected_yaw - theta if np.abs(phi)<2 else (phi-2*np.pi if phi>0 else phi+2*np.pi)

            self.si = s[i]
            self.dist = d_min+0.0001
            self.phi = phi_corrected
            self.vx = msg.vx
     

    def set_mpc_route(self, msg:Trajectory):
        if self.FIRST_LAP:

            route = np.array([[msg.trajectory[i].x,msg.trajectory[i].y] for i in range(0,len(msg.trajectory),2)])
            
            tck, u = splprep(route.T, s=3, per=True)  
            u_new = np.linspace(u.min(), u.max(), 1000)
            puntos_curva = np.array(splev(u_new, tck))
            self.x = puntos_curva[0]
            self.y = puntos_curva[1]


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
            self.s = s
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

            
