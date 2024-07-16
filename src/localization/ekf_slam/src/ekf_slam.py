import numpy as np
import rospy
from fssim_common.msg import State
from common_msgs.msg import Controls
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import time

class EKF_SLAM:
    def __init__(self):

        self.R = np.diag([0.002,0.002,0.0005]) # sigma_x, sigma_y, sigma_theta
        self.Q = np.diag([0.003,0.005]) # sigma_r, sigma_phi
        self.Fx = np.eye(3) # Used in both prediction and measurement updates
        self.prev_t = time.time()


        # Initialize robot state estimate and sigma
        self.mu = np.zeros((3,1)) # px, py, theta
        self.sigma = np.eye(3)*0.01
        self.sigma[2,2] = 0
    

        self.u = np.array([0,0])
        self.zs = None
        self.n_landmarks = 0
        self.threshold = 0.5

        self.state_pub = rospy.Publisher("/state_marker", Marker, queue_size=1)
        rospy.Subscriber('/controls', Controls, self.state_handle, queue_size=1)
        rospy.Subscriber('/perception_map', PointCloud2, self.perception_handle, queue_size=1)

    def state_handle(self, msg: Controls):
        self.u = np.array([msg.accelerator,msg.steering*np.pi/180])

    def perception_handle(self, msg: PointCloud2):
        self.prediction_update()

        
        map = point_cloud2.read_points(msg, field_names=("x", "y", "z","color","score"),skip_nans=True)
        for c in map:
            dist = np.sqrt((self.mu[0]+c[0])**2 + (self.mu[1]+c[1])**2)
            phi = self.mu[2] + np.arctan2(c[1],c[0])
            self.single_measurement_update(np.array([dist,phi]))
        

        # self.measurement_update()


    def prediction_update(self):

        dt = time.time()-self.prev_t
        self.prev_t = time.time()
        
        theta = self.mu[2]
        v,w = self.u[0],self.u[1]

        # Update state estimate mu with model
        state_model_mat = np.zeros((3,1)) # Initialize state update matrix from model
        state_model_mat[0] = -(v/w)*np.sin(theta)+(v/w)*np.sin(theta+w*dt) if w>0.01 else v*np.cos(theta)*dt # Update in the robot x position
        state_model_mat[1] = (v/w)*np.cos(theta)-(v/w)*np.cos(theta+w*dt) if w>0.01 else v*np.sin(theta)*dt # Update in the robot y position
        state_model_mat[2] = w*dt # Update for robot heading theta
        self.mu = self.mu + self.Fx.T @ state_model_mat # Update state estimate, simple use model with current state estimate

        # Update state uncertainty sigma
        state_jacobian = np.zeros((3,3)) # Initialize model jacobian
        state_jacobian[0,2] = (v/w)*np.cos(theta) - (v/w)*np.cos(theta+w*dt) if w>0.01 else -v*np.sin(theta)*dt # Jacobian element, how small changes in robot theta affect robot x
        state_jacobian[1,2] = (v/w)*np.sin(theta) - (v/w)*np.sin(theta+w*dt) if w>0.01 else v*np.cos(theta)*dt # Jacobian element, how small changes in robot theta affect robot y
        G = np.eye(self.sigma.shape[0]) + self.Fx.T @ state_jacobian @ self.Fx # How the model transforms uncertainty
        self.sigma = G @ self.sigma @ G.T + self.Fx.T @ self.R @ self.Fx # Combine model effects and stochastic noise

        # print(self.mu)
        # print(self.sigma)

    

    def single_measurement_update(self,landmark):
            dist = landmark[0]
            phi = landmark[1]

            x = self.mu[0]
            y = self.mu[1]
            yaw = self.mu[2]

            mu_landmark = np.array([x + dist*np.cos(phi+yaw),
                                    y+ dist*np.sin(phi+yaw)]) 
            
            d_min = np.Infinity
            index = None
            for i in range(self.n_landmarks):
                d = np.linalg.norm(self.mu[i+3:i+4]-mu_landmark)
                if d < d_min:
                    d_min = d
                    index = i
                    
            if d_min < self.threshold:
                pass 
            else:
                self.mu = np.vstack((self.mu, mu_landmark))
                self.Fx = np.pad(self.Fx, ((0, 0), (0, 2)), 'constant', constant_values=0)
                self.sigma = np.pad(self.sigma, ((0, 2), (0, 2)), 'constant', constant_values=0)
                self.sigma[-2,-2] = 0.001
                self.sigma[-1,-1] = 0.001
                self.n_landmarks+=1
                
            
            # delta  = mu_landmark - np.array([[x],[y]]) # Helper variable
            # q = np.linalg.norm(delta)**2 # Helper variable

            # dist_est = np.sqrt(q) # Distance between robot estimate and and landmark estimate, i.e., distance estimate
            # phi_est = np.arctan2(delta[1,0],delta[0,0])-theta; phi_est = np.arctan2(np.sin(phi_est),np.cos(phi_est)) # Estimated angled between robot heading and landmark
            # z_est_arr = np.array([[dist_est],[phi_est]]) # Estimated observation, in numpy array
            # z_act_arr = np.array([[dist],[phi]]) # Actual observation in numpy array
            # delta_zs[lidx] = z_act_arr-z_est_arr # Difference between actual and estimated observation

            # # Helper matrices in computing the measurement update
            # Fxj = np.block([[Fx],[np.zeros((2,Fx.shape[1]))]])
            # Fxj[n_state:n_state+2,n_state+2*lidx:n_state+2*lidx+2] = np.eye(2)
            # H = np.array([[-delta[0,0]/np.sqrt(q),-delta[1,0]/np.sqrt(q),0,delta[0,0]/np.sqrt(q),delta[1,0]/np.sqrt(q)],\
            #             [delta[1,0]/q,-delta[0,0]/q,-1,-delta[1,0]/q,+delta[0,0]/q]])
            # H = H.dot(Fxj)
            # Hs[lidx] = H # Added to list of matrices
            # Ks[lidx] = sigma.dot(np.transpose(H)).dot(np.linalg.inv(H.dot(sigma).dot(np.transpose(H)) + Q)) # Add to list of matrices



