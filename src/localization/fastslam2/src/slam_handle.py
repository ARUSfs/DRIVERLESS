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

N_LANDMARKS = 10
ASSOCIATION_THRESH = 0.4
P = np.eye(3, dtype=np.float32)  # Cov. matrix of (3)
P_inv = np.linalg.inv(P)         # Precomputing since it's contant.

class FastSLAM2:

    def __init__(self):
        self.position = np.zeros((3, 1), dtype=np.float32)  # [x, y, yaw]^t
        self.position_cov = np.eye(3, dtype=np.float32)  # \Sigma_s in [1]
        self.motion = np.zeros((2, 1), dtype=np.float32)  # Equivalent to u^t in [1]
        self.landmarks = np.zeros((N_LANDMARKS, 2), dtype=np.float32)  # \Theta^t in [1]
        self.landmarks_cov = np.zeros((N_LANDMARKS*2, 2), dtype=np.float32)  # \Sigma_\Theta in [1]

        self.last_rostime = 0

    def process

    def update_particle(self, delta_time):
        '''Updates the state of the particle with a bicycle model. Takes current linear velocity
        and yaw to update the particle's position. Corresponds to (3) in [1].
        '''

        B = np.array([[np.cos(self.position[2]), 0],
                      [np.sin(self.position[2]), 0],
                      [0,                        1]], dtype=np.float32)
        self.position += (B @ self.motion) * delta_time
        self.position[2] = bound_angle(self.position[2])

    def pose_sampling(self, Gs: np.ndarray, Gtheta: np.ndarray,
                      Q_inv: np.ndarray, delta_z: np.ndarray):
        '''Q defined in (15). Its an input parameter because the matrix is shared with
        landmark updating.
        delta_z: (z-\hat z)
        '''
        self.position_cov = np.linalg.inv(Gs.T @ G_inv @ Gs + P_inv)
        self.position += self.position_cov @ (Gs.T @ (Q_inv @ delta_z))
        # Right-to-left matmul is faster in this case since delta_z is a vector. Parenthesis are
        # ugly, but I am unsure of @ precedence and havent found much in numpy's doc.

    def update_with_observations(self):
        pass

    def calculate_jacobians(self, previous_landmark_position):
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
                            [ cosphi, -sinphi]], dtype=np.float32)
        Gtheta = Rt.copy()

        Gs = np.empty((2, 3), dtype=np.float32)
        Gs[:, :2] = Rt.copy()
        Gs[2, :] = Rtprime @ (previous_landmark_position - self.position[:2]).reshape((2, 1))

        return Gtheta, Gs


def bound_angle(angle: float):
    '''Bounds an angle between [-pi, pi] centered at 0'''
    return (angle + np.pi) % (2 * np.pi) - np.pi


