"""
Handle for FastSLAM2 implementation following
[1]: http://robots.stanford.edu/papers/Montemerlo03a.pdf
with M=1 particles.

We will only be concerned with the state in the current time, so all variables will refer to v^t
where t is now. For any notation doubts consult [1].

@author: Jacobo Pindado Perea
@date: 20230410
"""

import rospy
import numpy as np
from scipy.stats import multivariate_normal
import tf

from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from fssim_common.msg import State
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array

N_LANDMARKS = 200
ASSOCIATION_THRESH = 0.05
P = np.eye(3, dtype=np.float32)  # Cov. matrix of (3)
P_inv = np.linalg.inv(P)         # Precomputing since it's contant.
R = np.eye(2, dtype=np.float32)
TF_FRAME = 'SLAM_POS'


class FastSLAM2:

    def __init__(self):
        self.position = np.zeros(3, dtype=np.float32)  # [x, y, yaw]^t
        self.position_cov = P.copy()                        # \Sigma_s in [1]

        self.landmarks = np.zeros((N_LANDMARKS, 2), dtype=np.float32)  # \Theta^t in [1]
        self.landmarks_cov = np.zeros((N_LANDMARKS, 2, 2), dtype=np.float32)  # \Sigma_\Theta in [1]
        self.populated_landmarks = np.zeros(N_LANDMARKS, dtype=np.bool)

        self.motion = np.zeros(2, dtype=np.float32)  # Equivalent to u^t in [1]
        self.last_rostime = rospy.get_rostime().secs
        self.got_map = False

        self.br = tf.TransformBroadcaster()
        #rospy.Subscriber('/fssim/base_pose_ground_truth', State, self.state_callback)
        rospy.Subscriber('/camera/cones', PointCloud2, self.camera_callback)

    def process_map(self, observations: np.ndarray):
        for observation in observations:
            self.forward_observation(observation)

    def forward_observation(self, observation: np.ndarray):
        '''Will apply 4.4 -> 4.1 -> 4.2 for each observation.
        observation: [x, y, 1] position of observed landmark with respect to vehicle frame.
        '''
        observed_landmark = self.get_inv_observation_func() @ observation

        landmark_ind = self.get_corresponding_landmark(observed_landmark)
        Gtheta, Gs = self.calculate_jacobians(landmark_ind)

        # Equivalent to original \hat z in [1] since our g is linear with \theta.
        delta_z = observed_landmark - self.landmarks[landmark_ind, :]

        Q_inv = np.linalg.inv(R + Gtheta @ self.landmarks_cov[landmark_ind] @ Gtheta.T)

        self.pose_sampling(Gs, Q_inv, delta_z)

        self.update_landmark_estimate(Gs, Gtheta, Q_inv, delta_z, landmark_ind)

    def update_particle(self, delta_time: float):
        '''Updates the state of the particle with a bicycle model. Takes current linear velocity
        and yaw to update the particle's position. Corresponds to (3) in [1].
        '''
        if not self.got_map:
            return

        B = np.array([[np.cos(self.position[2]), 0],
                      [np.sin(self.position[2]), 0],
                      [0,                        1]], dtype=np.float32)
        self.position += (B @ self.motion.reshape(2, 1) * delta_time).flat
        self.position[2] = bound_angle(self.position[2])

    def pose_sampling(self, Gs: np.ndarray, Q_inv: np.ndarray, delta_z: np.ndarray):
        '''Q defined in (15). Its an input parameter because the matrix is shared with
        landmark updating.
        delta_z: (z-\hat z)
        '''
        self.position_cov = np.linalg.inv(Gs.T @ Q_inv @ Gs + P_inv)
        self.position += self.position_cov @ (Gs.T @ (Q_inv @ delta_z))
        # Right-to-left matmul is faster in this case since delta_z is a vector. Parenthesis are
        # ugly, but I am unsure of @ precedence and havent found much in numpy's doc.

    def update_landmark_estimate(self, Gs: np.ndarray, Gtheta: np.ndarray, Q_inv: np.ndarray,
                                 delta_z: np.ndarray, landmark_index: int):
        K = self.landmarks_cov[landmark_index] @ Gtheta.T @ Q_inv  # (16)

        self.landmarks[landmark_index, :] += K @ delta_z  # (17)
        self.landmarks_cov[landmark_index] -= K @ Gtheta @ self.landmarks_cov[landmark_index]  # (18)

    def get_corresponding_landmark(self, observed_landmark: np.ndarray):
        observation_probability = np.zeros(N_LANDMARKS, dtype=np.float32)
        for ind in np.nonzero(self.populated_landmarks)[0]:
            observation_probability[ind] = multivariate_normal.pdf(observed_landmark,
                                                                   mean=self.landmarks[ind, :]
                                                                            .flatten(),
                                                                   cov=self.landmarks_cov[ind, :, :]
                                                                           .reshape(2,2))

        max_prob_index = np.argmax(observation_probability)
        rospy.logwarn(self.landmarks[self.populated_landmarks])
        rospy.logwarn(observed_landmark)
        #rospy.logwarn((observation_probability[observation_probability>0]))
        #rospy.logwarn(self.landmarks_cov[self.populated_landmarks])
        if observation_probability[max_prob_index] < ASSOCIATION_THRESH:
            available_positions = np.nonzero(self.populated_landmarks == False)[0]
            if len(available_positions) == 0:
                rospy.logwarn('No available landmark indices in array')
                return max_prob_index
            else:
                new_lm_ind = available_positions[0]
                self.landmarks[new_lm_ind, :] = observed_landmark
                self.landmarks_cov[new_lm_ind] = R.copy()
                self.populated_landmarks[new_lm_ind] = True
                return new_lm_ind

        else:
            return max_prob_index


    def calculate_jacobians(self, landmark_index: int):
        '''Notation follows [1] using euclidean form of g where instead of distance and bearing
        we directly consider landmark position with respect to the car reference frame.

                    [[R^t, R^t*t]
        g(theta, s)= [0,0,   1  ] * [theta_x, theta_y, 1]^t

        with t the coordinates of the state, and R is the rotation matrix of the state's yaw.
        '''
        cosphi = np.cos(self.position[2])
        sinphi = np.sin(self.position[2])
        Rt = np.array([[cosphi, -sinphi],
                       [sinphi, cosphi]], dtype=np.float32)
        Rtprime = np.array([[-sinphi, -cosphi],
                            [cosphi , -sinphi]], dtype=np.float32)
        Gtheta = Rt.copy()

        Gs = np.empty((2, 3), dtype=np.float32)
        Gs[:, :2] = Rt.copy()
        Gs[:, 2] = (Rtprime @ (self.landmarks[landmark_index, :]
                               - self.position[:2]).reshape((2, 1))).flat

        return Gtheta, Gs

    def get_inv_observation_func(self):
        '''Matrix that changes reference frame from the car's frame to global frame.
        '''
        t_mat = np.empty((2, 3), dtype=np.float32)
        rmat = np.array([[np.cos(self.position[2]), -np.sin(self.position[2])],
                                  [np.sin(self.position[2]),  np.cos(self.position[2])]])
        t_mat[:2, :2] = rmat
        t_mat[:2, 2] = self.position[:2]
        return t_mat

    def state_callback(self, msg: State):
        self.motion[0] = np.linalg.norm([msg.vx, msg.vy])
        self.motion[1] = msg.r
        delta_time = msg.header.stamp.secs - self.last_rostime
        self.update_particle(delta_time)
        self.last_rostime = msg.header.stamp.secs

    def camera_callback(self, msg: PointCloud2):
        points = pointcloud2_to_xyz_array(msg)
        points[:, 2] = 1
        self.process_map(points)
        rospy.logerr(self.landmarks[(self.landmarks != 0).all(axis=1), :].shape)


def bound_angle(angle: float):
    '''Bounds an angle between [-pi, pi] centered at 0'''
    return (angle + np.pi) % (2 * np.pi) - np.pi
