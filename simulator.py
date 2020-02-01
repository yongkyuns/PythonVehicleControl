'''
.. module:: simulator
   :synopsis: Top-level simulator class for executing simulation
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module defines classes needed for executing simulation. All of the simulation-related aspects
(e.g. simulation step, updating graphics, executing and coordinating object movement) are done
by the simulator module.
'''

import vehicle
import visualizer as view
import numpy as np

try:
    import colored_traceback.always 
except:
    pass

# view_graph object keys for visualization
CAR = 'car'
LOCAL_PATH = 'local_path'
CTRL_PTS = 'ctrl_pts'
GLOBAL_PATH = 'global_path'
STR_ANG = 'str_ang'
REF1_ERR = 'ref1_err'

class Simulator:
    '''
    This class manages all aspects of simulation and owns data-related
    classes (e.g. vehicle) and view-related classes (e.g. visualizer).
    
    ================  ==================================================
    **Arguments:**
    dt                (float) sample time of the simulation. Changing sample time will update discrete dynamics of member objects.
    sim_time          (float) simulation time in seconds
    online_mode       (bool)  Specify visualization mode. If True, update graphics along with simualtion at runtime.

    **Variables:**
    currentStep       (int) Current simulation step
    view              (Visualizer) Main view for display              
    ================  ==================================================
    '''

    def __init__(self, dt=0.01, sim_time=10, online_mode=False):
        self._dt = dt # [sec]
        self.vehicle = vehicle.Vehicle(sample_time=self.sample_time,controller='PID')

        self.currentStep = 0
        self.sim_time = sim_time
        self._t = 0
        self.t_hist = []

        self.view = view.Visualizer(self.step,dx=self.sample_time, refresh_rate=50)

        self.view.graph[GLOBAL_PATH].setData(x=self.vehicle.path.x,y=self.vehicle.path.y)

    def run(self):
        '''
        Invoke entry_point of the Visualizer
        '''
        self.view.entry_point()

    def step(self):
        '''
        Execute 1 time step of simulation.
        '''
        i = self.currentStep
        self._t += self._dt

        self.vehicle.move()
        
        self.t_hist.append(self.t)

        log = self.vehicle.logger
        #3D Scene object update
        self.view.graph[CAR].setData(x=log.x[i],y=log.y[i],z_ang=log.yaw[i]*180/np.pi)
        self.view.graph[CTRL_PTS].setData(x=log.ctrl_pt_x[i], y=log.ctrl_pt_y[i])
        self.view.graph[LOCAL_PATH].setData(x=log.path_x[i], y=log.path_y[i])

        #2D Plot update
        self.view.graph[STR_ANG].setData(x=self.t_hist,y=log.str_ang)
        self.view.graph[REF1_ERR].setData(x=self.t_hist,y=log.ref)

        self.currentStep += 1


    @property
    def dt(self):
        return self._dt
    @dt.setter
    def sample_time(self,value):
        self._dt = value
        self.vehicle.update_sample_time(self._dt)
        self.view.dx = self._dt
    
    @property
    def t(self):
        return self._t



def main():
    
    sim = Simulator(sim_time=1)
    sim.run()

if __name__ == '__main__':
    main()



