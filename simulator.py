# import sys
# import os
# cwd = os.getcwd()
# print(cwd)
from controller import MPC, PID
import vehicle
import path_planner
import visualizer as view
import numpy as np

#temp
from pyqtgraph.Qt import QtGui
import time
import math
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
        
        self.N = 10000
        self.currentStep = 0

        ax = [0, 50, 100, 150, 200, 250]
        ay = [0, 0, 30, 60, 60, 60]

        self.view = view.Visualizer(self.step)
        self.path = path_planner.Planner(ax,ay)
        self.update_view_data(self.path.x,self.path.y,self.view.global_path)

        self.controller = PID(T=5,NY=2,P_Gain=1,I_Gain=0,D_Gain=0,weight_split_T=[0.5,0.5], weight_split_Y=[0.5,0.5])
        # self.controller = MPC(self.vehicle.get_dynamics_model)

        self.init_log()

    def update_view_data(self,x,y,plotObject):
        path_pts = np.vstack([x,y]).transpose()
        plotObject.setData(data=path_pts)

    def init_log(self):
        N = self.N
        self.output_hist = np.zeros((self.vehicle.model.C.shape[0],N))
        self.output_ref_hist = np.zeros((self.vehicle.model.C.shape[0],N))
        self.x_hist = np.zeros((N,1))
        self.y_hist = np.zeros((N,1))
        self.yaw_hist = np.zeros((N,1))

    def run(self):
        self.view.entry_point()

    def calcSteeringAng(self):
        # Update vehicle states
        pos_x = self.vehicle.x
        pos_y = self.vehicle.y
        yaw = self.vehicle.yaw
        Vx = self.vehicle.params.Vx
        
        # Generate local path to follow --> subset of global path near vehicle in relative coordinate frame
        rel_path_x, rel_path_y, rel_path_yaw = self.path.detect_local_path(pos_x,pos_y,yaw)
        step = Vx*self.controller.dt
        # Calculate reference output (lateral position, heading) from desired path
        x_ref, pos_ref, yaw_ref = self.path.calc_ref(rel_path_x, rel_path_y, rel_path_yaw, step, self.controller.T)
        yref = np.array([pos_ref,yaw_ref])

        # state = np.array([0,self.vehicle.states[1].item(),0,self.vehicle.states[3].item()])
        # str_ang = self.controller.control(yref,state)
        str_ang = self.controller.control(yref)

        #Update view
        self.update_view_data(x_ref,pos_ref,self.view.control_points)
        self.update_view_data(rel_path_x,rel_path_y,self.view.local_path)

        return yref, str_ang

    def step(self):
        i = self.currentStep

        yref, str_ang = self.calcSteeringAng()
        self.output_hist[:,i],self.x_hist[i,0],self.y_hist[i,0],self.yaw_hist[i,0] = self.vehicle.move(str_ang)
        self.output_ref_hist[:,i] = yref[:,0]

        x = self.x_hist[i,0]
        y = self.y_hist[i,0]
        yaw = self.yaw_hist[i,0]

        self.view.car.setData(x=x,y=y,z_ang=yaw*180/np.pi)

        self.currentStep += 1

    @property
    def sample_time(self):
        return self._sample_time
    @sample_time.setter
    def sample_time(self,value):
        self._sample_time = value
        self.vehicle.update_sample_time(self._sample_time)


def main():
    
    sim = Simulator()
    sim.view.entry_point()

if __name__ == '__main__':
    main()



