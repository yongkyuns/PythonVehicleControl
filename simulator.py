# import sys
# import os
# cwd = os.getcwd()
# print(cwd)
from controller import MPC, PID
import vehicle
import path_planner
import numpy as np

#temp
import time
import math
import matplotlib.pyplot as plt
import cvxpy
from scipy import sparse
from scipy import signal
import sys
sys.path.append('.')
from PythonRobotics.PathPlanning.CubicSpline import cubic_spline_planner as planner

try:
    import colored_traceback.always
except:
    pass



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

N = 1000 # Simulation time in steps

def main():

    sim = Simulator()

    ax = [0, 50, 100, 150, 200, 250]
    ay = [0, 0, 30, 60, 60, 60]

    path = path_planner.Planner(ax,ay)
    mpc_controller = MPC(sim.vehicle.get_dynamics_model)
    pid_controller = PID(T=5,NY=2,P_Gain=1,I_Gain=0,D_Gain=0,weight_split_T=[0.5,0.5], weight_split_Y=[0.5,0.5])

    output_hist = np.zeros((sim.vehicle.model.C.shape[0],N))
    output_ref_hist = np.zeros((sim.vehicle.model.C.shape[0],N))
    x_hist = np.zeros((N,1))
    y_hist = np.zeros((N,1))
    yaw_hist = np.zeros((N,1))

    for i in range(1,N):
        
        # Update vehicle states
        pos_x = sim.vehicle.x
        pos_y = sim.vehicle.y
        yaw = sim.vehicle.yaw
        Vx = sim.vehicle.params.Vx
        
        time_begin = time.time()
        # Generate local path to follow --> subset of global path near vehicle in relative coordinate frame
        rel_path_x, rel_path_y, rel_path_yaw = path.detect_local_path(pos_x,pos_y,yaw)
        step = Vx*mpc_controller.dt
        # Calculate reference output (lateral position, heading) from desired path
        pos_ref, yaw_ref = path.calc_ref(rel_path_x, rel_path_y, rel_path_yaw, step, mpc_controller.T)
        yref = np.array([pos_ref,yaw_ref])
        time_end = time.time()
        print('Path time    = {:.3f} sec'.format(time_end-time_begin))

        time_begin = time.time()
        # state = np.array([0,sim.vehicle.states[1].item(),0,sim.vehicle.states[3].item()])
        # str_ang = mpc_controller.control(yref,state)
        str_ang = pid_controller.control(yref)
        time_end = time.time()
        # print('Control time = {:.3f} sec'.format(time_end-time_begin))

        output_hist[:,i],x_hist[i,0],y_hist[i,0],yaw_hist[i,0] = sim.vehicle.move(str_ang)
        output_ref_hist[:,i] = yref[:,0]

        plt.cla()
        plt.plot(path.x,path.y)
        plt.plot(x_hist[0:i+1,0],y_hist[0:i+1,0])
        pos_x = x_hist[i,0]
        pos_y = y_hist[i,0]
        plot_car(pos_x,pos_y,yaw_hist[i,0],steer=str_ang)
        plt.axis("equal")
        plt.grid(True)
        window_size = 20
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



