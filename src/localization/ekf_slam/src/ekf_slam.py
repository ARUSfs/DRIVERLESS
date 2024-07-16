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

        # self.R = np.diag([0.002,0.002,0.0005]) # sigma_x, sigma_y, sigma_theta
        # self.Q = np.diag([0.003,0.005]) # sigma_r, sigma_phi
        self.R = np.diag([0.00,0.0,0.000]) # sigma_x, sigma_y, sigma_theta
        self.Q = np.diag([0.00,0.00]) # sigma_r, sigma_phi
        self.Fx = np.eye(3) # Used in both prediction and measurement updates
        self.prev_t = time.time()


        # Initialize robot state estimate and sigma
        self.mu = np.zeros((3,1)) # px, py, theta
        self.sigma = np.eye(3)*0.000000000000001
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
        
        self.measurement_update(map)
    


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

    

    def measurement_update(self,map):
        rospy.loginfo("############################################")
        x = self.mu[0][0]
        y = self.mu[1][0]
        yaw = self.mu[2][0] 
        # x = 0
        # y = 0
        # yaw = 0
        delta_zs = [np.zeros((2,1)) for _ in range(self.n_landmarks)] # A list of how far an actual measurement is from the estimate measurement
        Ks = [np.zeros((self.mu.shape[0],2)) for _ in range(self.n_landmarks)] # A list of matrices stored for use outside the measurement for loop
        Hs = [np.zeros((2,self.mu.shape[0])) for _ in range(self.n_landmarks)] # A list of matrices stored for use outside the measurement for loop
    
        for c in map:
            dist = np.sqrt(c[0]**2 + c[1]**2)
            phi = np.arctan2(c[1],c[0])

            mu_landmark = np.array([[x + dist*np.cos(phi+yaw)],
                                    [y+ dist*np.sin(phi+yaw)]]) 
            
            d_min = np.Infinity
            index = None
            for i in range(self.n_landmarks):
                d = np.sqrt((self.mu[i+3,0]-mu_landmark[0,0])**2 + (self.mu[i+4,0]-mu_landmark[1,0])**2)
                if d < d_min:
                    d_min = d
                    index = i
            if d_min < self.threshold:
                mu_landmark = self.mu[index+3:index+5]
            else:
                index = -1
                self.mu = np.vstack((self.mu, mu_landmark))
                self.Fx = np.pad(self.Fx, ((0, 0), (0, 2)), 'constant', constant_values=0)
                self.sigma = np.pad(self.sigma, ((0, 2), (0, 2)), 'constant', constant_values=0)
                self.sigma[-2,-2] = 0.0000001
                self.sigma[-1,-1] = 0.0000001
                self.n_landmarks+=1

                delta_zs.append(np.zeros((2,1)))
                Ks.append(np.zeros((self.mu.shape[0],2)))
                Hs.append(np.zeros((2,self.mu.shape[0])))
            
            
            delta  = mu_landmark - np.array([[x],[y]]) # Helper variable
            q = np.linalg.norm(delta)**2 # Helper variable

            dist_est = np.sqrt(q) # Distance between robot estimate and and landmark estimate, i.e., distance estimate
            phi_est = np.arctan2(delta[1,0],delta[0,0])-yaw
            phi_est = np.arctan2(np.sin(phi_est),np.cos(phi_est)) # Estimated angled between robot heading and landmark
            
            z_est_arr = np.array([[dist_est],[phi_est]]) # Estimated observation, in numpy array
            z_act_arr = np.array([[dist],[phi]]) # Actual observation in numpy array
            delta_zs[index] = z_act_arr-z_est_arr # Difference between actual and estimated observation
            
            # Helper matrices in computing the measurement update
            Fxj = np.block([[self.Fx],[np.zeros((2,self.Fx.shape[1]))]])
            Fxj[3:5,3+2*index:3+2*index+2] = np.eye(2)
            
            H = np.array([[-delta[0,0]/np.sqrt(q),-delta[1,0]/np.sqrt(q),0,delta[0,0]/np.sqrt(q),delta[1,0]/np.sqrt(q)],
                        [delta[1,0]/q,-delta[0,0]/q,-1,-delta[1,0]/q,+delta[0,0]/q]])
            H = H @ Fxj
            Hs[index] = H # Added to list of matrices
            Ks[index] = self.sigma @ H.T @ np.linalg.inv(H @ self.sigma @ H.T + self.Q) # Add to list of matrices
        
        # After storing appropriate matrices, perform measurement update of mu and sigma
        mu_offset = np.zeros(self.mu.shape) # Offset to be added to state estimate
        sigma_factor = np.eye(self.sigma.shape[0]) # Factor to multiply state uncertainty
        for index in range(self.n_landmarks):
            Ks[index] = np.pad(Ks[index], ((0, self.sigma.shape[0]-Ks[index].shape[0]),(0,0)), 'constant', constant_values=0)
            Hs[index] = np.pad(Hs[index], ((0,0),(0, self.sigma.shape[0]-Hs[index].shape[1])), 'constant', constant_values=0)
            
            mu_offset += Ks[index]@delta_zs[index] # Compute full mu offset
            sigma_factor -= Ks[index]@Hs[index] # Compute full sigma factor
            
        self.mu = self.mu + mu_offset # Update state estimate
        self.sigma = sigma_factor @ self.sigma # Update state uncertainty

        rospy.loginfo(self.mu[:3,0])
        rospy.loginfo(self.mu.shape)
       


