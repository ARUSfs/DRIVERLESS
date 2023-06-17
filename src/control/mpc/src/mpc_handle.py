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

@nb.njit(nb.types.Tuple([nb.float64, nb.float64[:]])(nb.float64[:]), fastmath=True)
def J_and_jacobian(q: np.ndarray):
    J_NUM_NORMS = 5
    J_VARS = [0, 1, 2, 8, 9]
    J_COSTS = [-10, 1, 1, 0, 1]
    temp_norms = np.zeros(J_NUM_NORMS, dtype=np.float64)
    jacobian = np.zeros(Qi_size*N, dtype=np.float64)
    for i in range(N):
        for j in range(J_NUM_NORMS):
            index = 10 * i + J_VARS[j]
            temp_norms[j] += q[index] * q[index]
            jacobian[index] = 2 * J_COSTS[j] * q[index]

    res = 0
    for i in range(J_NUM_NORMS):
        res += J_COSTS[i] * temp_norms[i]

    return (res, jacobian)

@nb.njit(nb.float64[:](nb.float64[:], nb.float64[:], nb.float64), fastmath=True)
def g(x: np.ndarray, u: np.ndarray, dt):
    if x[3] > 0:
        Fx = Cm*x[7] - pAcd*x[3]*x[3] - F_rodadura
    else:
        Fx = Cm*x[7] - pAcd*x[3]*x[3]
    ks = 0
    out = x.copy()
    sinphi = np.sin(x[2])
    cosphi = np.cos(x[2])
    out[0] += dt*(x[3]*cosphi - x[4]*sinphi)/(1-x[1]*ks)
    out[1] += dt*x[3]*sinphi + x[4]*cosphi
    out[2] += dt*x[5] - out[0]*ks
    out[3] += dt*Fx/MASS
    out[5] += dt*(x[6]*out[3] + u[0]*x[3])/WB # 4 depends on 5
    out[4] += dt*out[5]*lr
    out[6] += dt*u[0]*0
    out[7] += dt*u[1]
    return out

@nb.njit(nb.float64[:](nb.float64[:]))
def g_constraint(q: np.float64):
    equality_constraint = np.zeros(q.shape, dtype=np.float64)
    for i in range(1, N):
        xkm1 = q[(i-1)*Qi_size: i*Qi_size-2]
        ukm1 = q[i*Qi_size-2: i*Qi_size]
        xk = q[i*Qi_size: (i+1)*Qi_size-2]
        propagated_x = g(xkm1, ukm1, 0.1)
        equality_constraint = propagated_x - xk
    return equality_constraint


bounds = [(-10, 10),
          (-10, 10),
          (-10, 10),
          (-10, 10),
          (-10, 10),
          (-10, 10),
          (-10, 10),
          (-10, 10),
          (-10, 10),
          (-10, 10)]*N

constraints = {'type': 'eq',
               'fun':  g_constraint}

def find_optimum_control(previous_q: np.ndarray):
    initial_q = np.empty(previous_q.shape, dtype=np.float64)
    initial_q[:-10] = previous_q[10:].copy()
    initial_q[-2:0] = previous_q[-2:0]
    initial_q[-10:-2] = g(previous_q[-8:], previous_q[-2:], 0.1)
    optimum_control = minimize(J_and_jacobian, initial_q, method='SLSQP', jac=True, bounds=bounds)
    return optimum_control

