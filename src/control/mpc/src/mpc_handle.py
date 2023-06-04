from scipy.optimize import minimize
import numpy as np
import numba as nb

N = 2
Qi_size = 10
MASS = 200
Cm = 1
pAcd = 1
F_rodadura = 1
WB = 1
lr = 1

@nb.njit(nb.types.Tuple([nb.float32, nb.float32[:]])(nb.float32[:]), fastmath=True)
def J_and_jacobian(q: np.ndarray):
    J_NUM_NORMS = 5
    J_VARS = [0, 1, 2, 6, 7]
    J_COSTS = [1, 1, 1, 1, 1]
    temp_norms = np.zeros(J_NUM_NORMS, dtype=np.float32)
    jacobian = np.zeros(Qi_size*N, dtype=np.float32)
    for i in range(N):
        for j in range(J_NUM_NORMS):
            index = i*J_VARS[j]
            temp_norms[j] += q[index] * q[index]
            jacobian[index] = 2 * J_COSTS[j] * q[index]

    res = 0
    for i in range(J_NUM_NORMS):
        res += J_COSTS[i] * temp_norms[i]

    return (res, jacobian)

@nb.njit(nb.float32[:](nb.float32[:], nb.float32[:]), fastmath=True)
def g(x: np.ndarray, u: np.ndarray):
    Fx = Cm*x[7] - pAcd*x[3]*x[3] - F_rodadura
    ks = 0
    out = np.zeros(x.shape, dtype=np.float32)
    sinphi = np.sin(x[2])
    cosphi = np.cos(x[2])
    out[0] = (x[3]*cosphi - x[4]*sinphi)/(1-x[1]*ks)
    out[1] = x[3]*sinphi + x[4]*cosphi
    out[2] = x[5] - out[0]*ks
    out[3] = Fx/MASS
    out[5] = (x[6]*out[3] + u[0]*x[3])/WB # 4 depends on 5
    out[4] = out[5]*lr
    out[6] = u[0]
    out[7] = u[1]
    return out

@nb.njit(nb.float32[:](nb.float32[:]))
def g_constraint(q: np.float32):
    equality_constraint = np.zeros(q.shape, dtype=np.float32)
    for i in range(1, N):
        xkm1 = q[(i-1)*Qi_size: i*Qi_size-2]
        ukm1 = q[i*Qi_size-2: i*Qi_size]
        xk = q[i*Qi_size: (i+1)*Qi_size-2]
        propagated_x = g(xkm1, ukm1)
        equality_constraint = propagated_x - xk
    return equality_constraint


bounds = [(0, 1),
          (0, 1),
          (0, 1),
          (0, 1),
          (0, 1),
          (0, 1),
          (0, 1),
          (0, 1),
          (0, 1),
          (0, 1)]*N

constraints = {'type': 'eq',
               'fun':  g_constraint}

def find_optimum_control(previous_q: np.ndarray):
    initial_q = np.empty(previous_q.shape, dtype=np.float32)
    initial_q[:-7] = previous_q[7:]
    initial_q[-7:] = g(previous_q[-7:], np.zeros(2, dtype=np.float32))
    optimum_control = minimize(J_and_jacobian, initial_q, method='SLSQP', jac=True, bounds=bounds)

