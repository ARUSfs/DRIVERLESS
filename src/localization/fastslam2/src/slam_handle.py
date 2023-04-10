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

class FastSLAM2:

    def __init__(self):
        self.position = np.zeros((3, 1), dtype=np.float32)
        self.position_cov = np.eye(3, dtype=np.float32)
        self.motion = np.zeros((2, 1), dtype=np.float32)  # Equivalent to u^t in [1]
        self.landmarks = np.zeros((N_LANDMARKS, 2), dtype=np.float32)  # \Theta^t in [1]
        self.landmarks_cov = np.zeros((N_LANDMARKS*2, 2), dtype=np.float32)  # \Sigma_\Theta in [1]

        self.last_rostime = 0

    def update_particle(self, delta_time):
        B = np.array([[np.cos(self.position[2]), 0],
                      [np.sin(self.position[2]), 0],
                      [0,                        1]], dtype=np.float32)
        self.position += B @ self.motion
        self.position[2] = bound_angle(self.position[2])

    def update_with_observations(self):
        pass


def bound_angle(angle: float):
    '''Transforms an angle between [-pi, pi] centered at 0'''
    return (angle + np.pi) % (2 * np.pi) - np.pi


