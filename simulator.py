# import sys
# import os
# cwd = os.getcwd()
# print(cwd)
import vehicle
import numpy as np

#temp
from helper import *
import math
import matplotlib.pyplot as plt
import cvxpy
from scipy import sparse
import sys
sys.path.append('.')
from PythonRobotics.PathPlanning.CubicSpline import cubic_spline_planner as planner

try:
    import colored_traceback.always
except:
    pass

NU = 1
NX = 4
NY = 2
T = 5
DT = 0.1
DL = 0.1



class Simulator:
    def __init__(self):
        self._sample_time = 0.01 # [sec]
        self.vehicle = vehicle.Vehicle(sample_time=self.sample_time)

    @property
    def sample_time(self):
        return self._sample_time
    @sample_time.setter
    def sample_time(self,value):
        self._sample_time = value
        self.vehicle.update_sample_time(self._sample_time)


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    # Vehicle parameters
    LENGTH = 4.5  # [m]
    WIDTH = 2.0  # [m]
    BACKTOWHEEL = 1.0  # [m]
    WHEEL_LEN = 0.3  # [m]
    WHEEL_WIDTH = 0.2  # [m]
    TREAD = 0.7  # [m]
    WB = 2.5  # [m]

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")



def calc_nearest_index(x_pos, y_pos, cx, cy, cyaw, search_ref_idx=0, search_range=10):
    search_begin_idx = max(0,search_ref_idx - search_range)
    search_end_idx = min(len(cx) - 1, search_ref_idx + search_range)

    dx = [x_pos - icx for icx in cx[search_begin_idx:search_end_idx]]
    dy = [y_pos - icy for icy in cy[search_begin_idx:search_end_idx]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + search_begin_idx

    mind = math.sqrt(mind)

    dxl = cx[ind] - x_pos
    dyl = cy[ind] - y_pos

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind    


def calc_ref_output(pos_x, pos_y, yaw, Vx,  cx, cy, cyaw, pind):
    """
    Compute reference outputs [y, yaw] for horizon T
    """

    yref = np.zeros((NY, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(pos_x, pos_y, cx, cy, cyaw, search_ref_idx=pind)

    if pind >= ind:
        ind = pind

    pos_error = __calc_pos_ref(pos_x,pos_y,cx,cy,ind)
    yaw_error = __calc_yaw_ref(yaw,cyaw,ind)

    yref[0, 0] = pos_error
    yref[1, 0] = yaw_error

    travel = 0.0

    for i in range(T + 1):
        travel += abs(Vx) * DT
        dind = int(round(travel / DL))

        if (ind + dind) < ncourse:
            yref[0, i] = __calc_pos_ref(pos_x,pos_y,cx,cy,ind+dind)
            yref[1, i] = __calc_yaw_ref(yaw,cyaw,ind+dind)
        else:
            yref[0, i] = __calc_pos_ref(pos_x,pos_y,cx,cy,ncourse-1)
            yref[1, i] = __calc_yaw_ref(yaw,cyaw,ncourse-1)

    return yref, ind

def __calc_pos_ref(pos_x,pos_y,cx,cy,ind):
    return math.sqrt((cx[ind]-pos_x)**2+(cy[ind]-pos_y)**2)

def __calc_yaw_ref(yaw,cyaw,ind):
    return cyaw[ind] - yaw

def main():

    sim = Simulator()

    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]

    cx, cy, cyaw, _, _ = planner.calc_spline_course(ax, ay, ds=DL)

    x_pos = sim.vehicle.x
    y_pos = sim.vehicle.y
    target_ind, _ = calc_nearest_index(x_pos,y_pos, cx, cy, cyaw)
    

    # Constraints
    umax = np.array([1,1]) * 1
    umin = -umax
    urmax = np.array([1, 1, 1, 1]) * 1
    urmin = -urmax

    # Objective function
    Q = sparse.diags([10, 10])
    R = sparse.eye(1)   


    N = 1000
    output_hist = np.zeros((sim.vehicle.model.C.shape[0],N))
    output_ref_hist = np.zeros((sim.vehicle.model.C.shape[0],N))
    x_hist = np.zeros((N,1))
    y_hist = np.zeros((N,1))
    yaw_hist = np.zeros((N,1))



    pos_ref = np.linspace(0,10,N).reshape(1,N)
    yaw_ref = np.zeros((1,N))
    yr = np.concatenate((pos_ref,yaw_ref))


    for i in range(1,N):
        pos_x = sim.vehicle.x
        pos_y = sim.vehicle.y
        yaw = sim.vehicle.yaw
        Vx = sim.vehicle.params.Vx
        yref, target_ind = calc_ref_output(pos_x, pos_y, yaw, Vx, cx, cy, cyaw, target_ind)



        # Define problem
        u = cvxpy.Variable((NU, T+1))
        x = cvxpy.Variable((NX, T+1))
        y = cvxpy.Variable((NY, T+1))
        x_init = cvxpy.Parameter(NX)
        x_init.value = sim.vehicle.states.flatten()
        objective = 0
        constraints = [x[:,0] == x_init]

        for k in range(T):
            A = sim.vehicle.model.A
            B = sim.vehicle.model.B
            C = sim.vehicle.model.C
            objective += cvxpy.quad_form(y[:,k] - yref[:,k], Q) + cvxpy.quad_form(u[:,k], R)
            # objective += cvxpy.quad_form(y[:,k] - np.array([10,10]), Q) + cvxpy.quad_form(u[:,k], R)

            constraints += [x[:,k+1] == A*x[:,k] + B*u[:,k]]
            constraints += [y[:,k] == C*x[:,k]]
            # constraints += [urmin <= u[:,k+1] - u[:,k], u[:,k+1] - u[:,k] <= urmax]
            # constraints += [xmin <= x[:,k], x[:,k] <= xmax]
            # constraints += [umin <= u[:,k], u[:,k] <= umax]
        objective += cvxpy.quad_form(y[:,T] - yref[:,T], Q)
        # objective += cvxpy.quad_form(y[:,T] - yr[:,i], Q)
        prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)
        prob.solve(solver=cvxpy.OSQP, warm_start=True)

        str_ang = u[:,0].value
        # str_ang = 10*math.pi/180
        output_hist[:,i],x_hist[i,0],y_hist[i,0],yaw_hist[i,0] = sim.vehicle.move(str_ang)
        output_ref_hist[:,i] = yref[:,0]

        plt.cla()
        plt.plot(cx,cy)
        plt.plot(x_hist[0:i+1,0],y_hist[0:i+1,0])
        pos_x = x_hist[i,0]
        pos_y = y_hist[i,0]
        plot_car(pos_x,pos_y,yaw_hist[i,0],steer=str_ang)
        plt.axis("equal")
        plt.grid(True)
        window_size = 10
        plt.xlim(pos_x-window_size,pos_x+window_size)
        plt.ylim(pos_y-window_size,pos_y+window_size)
        plt.pause(0.00001)

    plt.figure()
    plt.plot(output_ref_hist[0,:], label="ref. pos.")
    plt.plot(output_hist[0,:], label='pos.')
    plt.plot(output_ref_hist[1,:], label="ref. yaw")
    plt.plot(output_hist[1,:], label='yaw')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()



