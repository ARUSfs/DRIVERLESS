"""
Handle for FastSLAM2 implementation following
[1]: http://robots.stanford.edu/papers/Montemerlo03a.pdf
[2]: http://robots.stanford.edu/papers/Thrun03g.pdf
with M=1 particles.

We will only be concerned with the state in the current time, so all variables will refer to v^t
where t is now. For any notation doubts consult [1].

@author: Jacobo Pindado Perea
@date: 20230410
"""

import rospy
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from numba import jit, prange, float32
from scipy.stats import multivariate_normal
import tf
from threading import Lock

from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from fssim_common.msg import State
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array


import time

N_LANDMARKS = 400
N_PARTICLES = 3
P = 0.01*np.eye(3, dtype=np.float32)  # Cov. matrix of (3)
P[2, 2] = 0.001
P_inv = np.linalg.inv(P)         # Precomputing since it's contant.
R = 0.1*np.eye(2, dtype=np.float32)
TF_FRAME = 'SLAM_POS'


class FastSLAM2:

    def __init__(self):
        self.position = np.zeros((N_PARTICLES, 3), dtype=np.float32)  # [x, y, yaw]^t
        self.position_cov = np.empty((3, 3), dtype=np.float32)        # \Sigma_s in [1]
        self.particle_weights = np.zeros(N_PARTICLES, dtype=np.float32)
        self.weighted_position = np.zeros(3, dtype=np.float32)
        self.lock = Lock()

        self.landmarks = np.zeros((N_PARTICLES, N_LANDMARKS, 2), dtype=np.float32)  # \Theta^t in [1]
        self.landmarks_cov = np.zeros((N_PARTICLES, N_LANDMARKS, 2, 2), dtype=np.float32)  # \Sigma_\Theta in [1]
        self.populated_landmarks = np.zeros(N_LANDMARKS, dtype=np.bool)
        self.weighted_landmarks = np.zeros((N_LANDMARKS, 2), dtype=np.float32)

        self.motion = np.zeros(2, dtype=np.float32)  # Equivalent to u^t in [1]
        self.last_rostime = rospy.get_rostime().secs
        self.got_map = False
        self.allow_pose_sampling = False

        rospy.Subscriber('/fssim/base_pose_ground_truth', State, self.state_callback, queue_size=20)
        rospy.Subscriber('/camera/cones', PointCloud2, self.camera_callback, queue_size=1)
        self.tf_broad = tf.TransformBroadcaster()
        self.p = rospy.Publisher('/landmarks', MarkerArray, queue_size=1)

        self.vels = list()

        self.i = 0


    def process_map(self, observations: np.ndarray):
        t = time.time()
        if self.i == 5:
            self.allow_pose_sampling = True
            rospy.logerr('='*100)
        self.i += 1
        with self.lock:
            for observation in observations:
                self.forward_observation(observation)
        self.got_map = True
        self.pub_markers()
        rospy.logwarn(f'{np.nonzero(self.populated_landmarks)[0].shape}, {1/(time.time() - t)}')

    def forward_observation(self, observation: np.ndarray):
        '''Will apply 4.4 -> 4.1 -> 4.2 for each observation.
        observation: [x, y, 1] position of observed landmark with respect to vehicle frame.
        '''

        observed_landmark = self.get_inv_observation_func() @ observation
        landmark_ind = self.get_corresponding_landmark(observed_landmark)

        for i in range(N_PARTICLES):
            Gtheta, Gs = self.calculate_jacobians(i, landmark_ind)

            # Equivalent to original \hat z in [1] since our g is linear with \theta.
            delta_z = observed_landmark - self.landmarks[i, landmark_ind, :]

            Q = R + Gtheta @ self.landmarks_cov[i, landmark_ind] @ Gtheta.T
            Q_inv = np.linalg.inv(Q)

            if self.allow_pose_sampling:
                self.pose_sampling(Gs, Q_inv, delta_z, i)

            self.update_landmark_estimate(Gs, Gtheta, Q_inv, delta_z, landmark_ind, i)

            self.particle_weights[i] = self.get_particle_weight(i, Q, Gs, delta_z)
            self.particle_weights /= self.particle_weights.sum()

            self.weighted_landmarks = np.sum(self.landmarks * self.particle_weights.reshape(-1, 1, 1), axis=0)
            self.weighted_position = np.sum(self.position * self.particle_weights.reshape(-1, 1), axis=0)
            self.weighted_position[2] = bound_angle(self.weighted_position[2])
        self.update_frame()

    def update_particle(self, delta_time: float):
        '''Updates the state of the particle with a bicycle model. Takes current linear velocity
        and yaw to update the particle's position. Corresponds to (3) in [1].
        '''
        if not self.got_map:
            return

        B = np.zeros((N_PARTICLES, 3, 2), dtype=np.float32)
        B[:, 0, 0] = np.cos(self.position[:, 2])
        B[:, 1, 0] = np.sin(self.position[:, 2])
        B[:, 2, 1] = 1

        with self.lock:
            self.position += delta_time * (np.random.multivariate_normal([0, 0, 0], P, size=N_PARTICLES) + B @ self.motion)
            for i in prange(N_PARTICLES): # TODO: Change to apply_along_axis, prange
                self.position[i, 2] = bound_angle(self.position[i, 2])
            self.weighted_position = np.sum(self.position * self.particle_weights.reshape(-1, 1), axis=0)

    def pose_sampling(self, Gs: np.ndarray, Q_inv: np.ndarray, delta_z: np.ndarray, particle: int):
        '''Q defined in (15). Its an input parameter because the matrix is shared with
        landmark updating.
        delta_z: (z-\hat z)
        '''
        self.position_cov = np.linalg.inv(Gs.T @ Q_inv @ Gs + P_inv)
        self.position[particle] += self.position_cov @ (Gs.T @ (Q_inv @ delta_z))
        # Right-to-left matmul is faster in this case since delta_z is a vector. Parenthesis are
        # ugly, but I am unsure of @ precedence and havent found much in numpy's doc.

    def update_landmark_estimate(self, Gs: np.ndarray, Gtheta: np.ndarray, Q_inv: np.ndarray,
                                 delta_z: np.ndarray, landmark_index: int, particle: int):
        K = self.landmarks_cov[particle, landmark_index] @ Gtheta.T @ Q_inv  # (16)

        self.landmarks[particle, landmark_index, :] += K @ delta_z  # (17)
        self.landmarks_cov[particle, landmark_index] -= K @ Gtheta @ self.landmarks_cov[particle, landmark_index]  # (18)

    def get_corresponding_landmark(self, observed_landmark: np.ndarray):
        distances = np.full(N_LANDMARKS, np.Inf)
        distances[self.populated_landmarks] = np.linalg.norm(self.weighted_landmarks[self.populated_landmarks] - observed_landmark,
                                                             axis=1)
        min_dist_index = np.argmin(distances)
        if not self.got_map or distances[min_dist_index] > 2:
            available_positions = np.nonzero(self.populated_landmarks == False)[0]
            if len(available_positions) == 0:
                rospy.logwarn('No available landmark indices in array')
                return min_dist_index
            else:
                new_lm_ind = available_positions[0]
                self.landmarks[:, new_lm_ind, :] = observed_landmark
                self.landmarks_cov[:, new_lm_ind] = R.copy()
                self.populated_landmarks[new_lm_ind] = True
                return new_lm_ind
        else:
            return min_dist_index

    def calculate_jacobians(self, particle_index:int, landmark_index: int):
        '''Notation follows [1] using euclidean form of g where instead of distance and bearing
        we directly consider landmark position with respect to the car reference frame.

                    [[R^t, R^t*t]
        g(theta, s)= [0,0,   1  ] * [theta_x, theta_y, 1]^t

        with t the coordinates of the state, and R is the rotation matrix of the state's yaw.
        '''
        cosphi = np.cos(self.position[particle_index, 2])
        sinphi = np.sin(self.position[particle_index, 2])
        Rt = np.array([[cosphi, -sinphi],
                       [sinphi, cosphi]], dtype=np.float32)
        Rprime = np.array([[-sinphi, cosphi],
                            [-cosphi , -sinphi]], dtype=np.float32)
        Gtheta = Rt.copy()

        Gs = np.empty((2, 3), dtype=np.float32)
        Gs[:, :2] = -Rt.copy()
        Gs[:, 2] = (Rprime @ (self.landmarks[particle_index, landmark_index, :]
                               - self.position[particle_index, :2]).reshape((2, 1))).flat

        return Gtheta, Gs

    def get_inv_observation_func(self):
        '''Matrix that changes reference frame from the car's frame to global frame.
        '''
        t_mat = np.empty((2, 3), dtype=np.float32)
        rmat = np.array([[np.cos(self.weighted_position[2]), -np.sin(self.weighted_position[2])],
                        [ np.sin(self.weighted_position[2]),  np.cos(self.weighted_position[2])]])
        t_mat[:2, :2] = rmat
        t_mat[:2, 2] = self.weighted_position[:2]
        return t_mat

    def get_particle_weight(self, particle: int, Q: np.ndarray, Gs: np.ndarray, delta_z):
        L = Q + Gs @ P @ Gs.T  # (60) in [2]
        # L is symmetric and pos. def. given that it is a cov. matrix. We can more efficiently calculate
        # the distribution using Cholesky.
        L_chol = np.linalg.cholesky(L)
        L_chol_det = L_chol.diagonal().prod()
        Ldz = L_chol @ delta_z
        return np.exp(-0.5*Ldz.T @ Ldz)/(np.sqrt(2*np.pi) * L_chol_det)

    def state_callback(self, msg: State):
        delta_time = msg.header.stamp.to_sec() - self.last_rostime
        self.last_rostime = msg.header.stamp.to_sec()
        if not self.allow_pose_sampling:
            return
        self.motion[0] = np.linalg.norm([msg.vx, msg.vy]) - 0.010
        self.motion[1] = msg.r
        self.update_particle(delta_time)
        self.update_frame()

    def camera_callback(self, msg: PointCloud2):
        points = pointcloud2_to_xyz_array(msg)
        points[:, 2] = 1
        self.process_map(points)

    def pub_markers(self):
        marray = MarkerArray()
        m1 = Marker()
        m1.action = Marker.DELETEALL
        marray.markers.append(m1)
        for i, lm in enumerate(self.weighted_landmarks[self.populated_landmarks]):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time().now()
            marker.type = Marker.CYLINDER
            marker.action = Marker.MODIFY
            marker.pose.position.x = lm[0]
            marker.pose.position.y = lm[1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1.0

            marker.lifetime = rospy.Duration()

            marray.markers.append(marker)
            marker.lifetime = rospy.Duration()
            marray.markers.append(marker)

        self.p.publish(marray)

    def update_frame(self):
        self.tf_broad.sendTransform((self.weighted_position[0], self.weighted_position[1], 0),
                                    tf.transformations.quaternion_from_euler(0,
                                                                             0,
                                                                             self.weighted_position[2]),
                                    rospy.Time.now(),
                                    'coche',
                                    'map')

@jit(float32(float32), nopython=True, nogil=True, cache=True)
def bound_angle(angle: float):
    '''Bounds an angle between [-pi, pi] centered at 0'''
    return (angle + np.pi) % (2 * np.pi) - np.pi
