from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse
import time
from scipy import signal

import matplotlib.pyplot as plt

try:
    import colored_traceback.always
except:
    pass

# Discrete time model of a quadcopter
Vx=60
m=1600
Iz=1705
lf=1.0
lr=1.0
Caf=66243
Car=66243

A = np.array([[-(2*Caf + 2*Car)/(m*Vx),       0,  -Vx-(2*Caf*lf-2*Car*lr)/(m*Vx),     0],
                [        0,                     0,              1,                      0],
                [-(2*lf*Caf-2*lr*Car)/(Iz*Vx),  0,  -(2*lf**2*Caf+2*lr**2*Car)/(Iz*Vx), 0],
                [        1,                     Vx,             0,                      0]])

B = np.array([[  2*Caf/m  ],
              [     0     ],
              [2*lf*Caf/Iz],
              [     0     ]])

C = np.array([[0, 0, 0, 1], # lateral position
              [0, 1, 0, 0]]) # yaw angle

D = np.array(0)
sample_time = 0.1
Ad,Bd,Cd,Dd,_ = signal.cont2discrete((A,B,C,D),sample_time)

nx = 4
nu = 1
ny = 2

# Constraints
umax = np.array([1,1]) * 1
umin = -umax
urmax = np.array([1, 1, 1, 1]) * 1
urmin = -urmax

# xmin = np.array([-np.pi/6,-np.pi/6,-np.inf,-np.inf,-np.inf,-1.,
#                  -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
# xmax = np.array([ np.pi/6, np.pi/6, np.inf, np.inf, np.inf, np.inf,
#                   np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])

# Objective function
Q = sparse.diags([10, 10])
R = 0.1*sparse.eye(1)

# Prediction horizon
N = 5

# Simulate in closed loop
nsim = 30
x_hist = np.zeros((nx,nsim))
u_hist = np.zeros((nu,nsim))

# Initial and reference states
x0 = np.zeros(4)
pos_ref = np.linspace(0,10,nsim).reshape(1,nsim)
yaw_ref = np.zeros((1,nsim))
yr = np.concatenate((pos_ref,yaw_ref))






for i in range(nsim):

    t_begin = time.time()
    # Define problem
    u = Variable((nu, N+1))
    x = Variable((nx, N+1))
    y = Variable((ny, N+1))
    x_init = Parameter(nx)
    x_init.value = x0
    objective = 0
    constraints = [x[:,0] == x_init]
    for k in range(N):
        objective += quad_form(y[:,k] - yr[:,i], Q) + quad_form(u[:,k], R)
        constraints += [x[:,k+1] == Ad*x[:,k] + Bd*u[:,k]]
        constraints += [y[:,k] == Cd*x[:,k]]
        # constraints += [urmin <= u[:,k+1] - u[:,k], u[:,k+1] - u[:,k] <= urmax]
        # constraints += [xmin <= x[:,k], x[:,k] <= xmax]
        # constraints += [umin <= u[:,k], u[:,k] <= umax]
    objective += quad_form(y[:,N] - yr[:,i], Q)
    prob = Problem(Minimize(objective), constraints)

    prob.solve(solver=OSQP, warm_start=True)

    t_end = time.time()
    print('{0:.3f}'.format(t_end - t_begin))

    x0 = Ad.dot(x0) + Bd.dot(u[:,0].value)
    x_hist[:,i] = x0
    u_hist[:,i] = u[:,0].value



plt.figure()
plt.subplot(311)
plt.plot(pos_ref[0,:])
plt.plot(x_hist[3,:])
plt.grid(True)

plt.subplot(312)
plt.plot(yaw_ref[0,:])
plt.plot(x_hist[1,:])
plt.grid(True)

plt.subplot(313)
for i in range(nu):
    plt.plot(u_hist[i,:])
plt.grid(True)
plt.show()

