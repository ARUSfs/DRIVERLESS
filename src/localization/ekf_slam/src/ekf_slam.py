import numpy as np
import rospy
from fssim_common.msg import State
from common_msgs.msg import Controls
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
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


        self.state_pub = rospy.Publisher("/state_marker", Marker, queue_size=1)
        rospy.Subscriber('/controls', Controls, self.state_handle, queue_size=1)
        rospy.Subscriber('/perception_map', PointCloud2, self.perception_handle, queue_size=1)

    def state_handle(self, msg: Controls):
        self.u = np.array([msg.accelerator,msg.steering*np.pi/180])

    def perception_handle(self, msg: PointCloud2):
        self.prediction_update()

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

        print(self.mu)
        print(self.sigma)


