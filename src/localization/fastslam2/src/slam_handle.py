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

N_LANDMARKS = 10
ASSOCIATION_THRESH = 0.4
P = np.eye(3, dtype=np.float32)  # Cov. matrix of (3)
P_inv = np.linalg.inv(P)         # Precomputing since it's contant.
R = np.linalg.eye(3, dtype=np.float32)


class FastSLAM2:

    def __init__(self):
        self.position = np.zeros((3, 1), dtype=np.float32)  # [x, y, yaw]^t
        self.position_cov = P.copy()                        # \Sigma_s in [1]

        self.landmarks = np.zeros((2, N_LANDMARKS), dtype=np.float32)  # \Theta^t in [1]
        self.landmarks_cov = np.zeros((2, N_LANDMARKS*2), dtype=np.float32)  # \Sigma_\Theta in [1]
        # TODO: Transform landmarks_cov into 3d array to prevent cumbersome slicing.
        self.populated_landmarks = np.array(N_LANDMARKS, dtype=np.bool)

        self.motion = np.zeros((2, 1), dtype=np.float32)  # Equivalent to u^t in [1]
        self.last_rostime = 0

    def forward_observation(self, observation: np.ndarray, delta_time: float):
        '''Will apply 4.4 -> 4.1 -> 4.2 for each observation.
        observation: [x, y, 1] position of observed landmark with respect to vehicle frame.
        '''
        self.update_particle(delta_time)

        inv_observation_func = self.get_inv_observation_func()

        landmark_ind = self.get_corresponding_landmark(observation, inv_observation_func)
        Gtheta, Gs = self.calculate_jacobians(landmark_ind)

        # Equivalent to original \hat z in [1] since our g is linear with \theta.
        delta_z = inv_observation_func @ observation - self.landmarks[:, landmark_ind]

        cov_slice = (slice(-1), slice(2*landmark_index, 2*(landmark_index + 1)))
        Q_inv = R + Gtheta @ self.landmarks_cov[cov_slice] @ Gtheta.T

        self.pose_sampling(Gs, Q_inv, delta_z)

        self.update_landmark_estimate(Gs, Gtheta, Q_inv, delta_z, landmark_ind)

    def update_particle(self, delta_time: float):
        '''Updates the state of the particle with a bicycle model. Takes current linear velocity
        and yaw to update the particle's position. Corresponds to (3) in [1].
        '''

        B = np.array([[np.cos(self.position[2]), 0],
                      [np.sin(self.position[2]), 0],
                      [0,                        1]], dtype=np.float32)
        self.position += (B @ self.motion) * delta_time
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
        cov_slice = (slice(-1), slice(2*landmark_index, 2*(landmark_index + 1)))
        K = self.landmarks_cov[cov_slice] @ Gtheta.T @ Q_inv  # (16)

        self.landmarks[:, landmark_index] += K @ delta_z  # (17)
        self.landmarks_cov[cov_slice] -= K @ Gtheta @ self.landmarks_cov[cov_slice]  # (18)

    def get_corresponding_landmark(self, observation: np.ndarray, inv_observation_mat: np.ndarray):
        predicted_coordinates = inv_observation_mat @ observation

        observation_probability = np.zeros(N_LANDMARKS, dtype=np.float32)
        for ind in np.nonzero(self.populated_landmarks):
            cov_slice = (slice(-1), slice(2*ind, 2*(ind + 1)))
            observation_probability = multivariate_normal.pdf(predicted_coordinates,
                                                              mean=self.landmarks[:, i],
                                                              cov=self.landmarks_cov[cov_slice])

        max_prob_index = np.argmax(observation_probability)

        if observation_probability[max_prob_index] < ASSOCIATION_THRESH:
            # TODO: Create new landmark
            pass
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
        Gs[2, :] = Rtprime @ (self.landmarks[:, i] - self.position[:2]).reshape((2, 1))

        return Gtheta, Gs

    def get_inv_observation_func(self):
        '''Matrix that changes reference frame from the car's frame to global frame.
        '''
        t_mat = np.empty((2, 3), dtype=np.float32)
        t_mat[:2, :2] = np.array([[np.cos(self.position[2]), -np.sin(self.position[2])],
                                  [np.sin(self.position[2]),  np.cos(self.position[2])]])
        t_mat[:2, 2] = self.position[:2]
        return t_mat


def bound_angle(angle: float):
    '''Bounds an angle between [-pi, pi] centered at 0'''
    return (angle + np.pi) % (2 * np.pi) - np.pi
