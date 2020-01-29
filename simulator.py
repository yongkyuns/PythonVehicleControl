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

class Simulator:
    '''
    This class manages all aspects of simulation and owns data-related
    classes (e.g. vehicle) and view-related classes (e.g. visualizer).
    
    ================  ==================================================
    **Arguments:**
    sample_time       (float) sample time of the simulation. Changing sample time will update discrete dynamics of member objects.
    
    **Variables:**
    N                 (int) Maximum log data size
    currentStep       (int) Current simulation step
    view              (Visualizer) Main view for display              
    ================  ==================================================
    '''

    def __init__(self):
        self._sample_time = 0.01 # [sec]
        self.vehicle = vehicle.Vehicle(sample_time=self.sample_time,controller='PID')
        
        self.N = 10000
        self.currentStep = 0
        self._t = 0
        self.t_hist = []

        self.view = view.Visualizer(self.step,dx=self.sample_time)

        self.update_view_data(self.vehicle.path.x,self.vehicle.path.y,self.view.graph['global_path'])


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
        self._t += self._sample_time

        self.vehicle.move()

        self.t_hist.append(self.t)

        log = self.vehicle.logger
        self.view.graph['car'].setData(x=log.x[i],y=log.y[i],z_ang=log.yaw[i]*180/np.pi)
        self.view.graph['str_ang'].setData(x=self.t_hist,y=log.str_ang)
        self.view.graph['ref1_err'].setData(x=self.t_hist,y=log.ref)
        self.update_view_data(log.ctrl_pt_x[i],log.ctrl_pt_y[i],self.view.graph['ctrl_pts'])
        self.update_view_data(log.path_x[i],log.path_y[i],self.view.graph['local_path'])

        self.currentStep += 1

    def update_view_data(self,x,y,plotObject):
        '''
        Update the data for the line&scatter plot items
        
        ================  ==================================================
        **Arguments:**
        x                 (numpy array) 1-by-n array of x coordinates
        y                 (numpy array) 1-by-n array of y coordinates
        plotObject        (object) object within Visualizer for update
        ================  ==================================================
        '''
        path_pts = np.vstack([x,y]).transpose()
        plotObject.setData(data=path_pts)

    @property
    def sample_time(self):
        return self._sample_time
    @sample_time.setter
    def sample_time(self,value):
        self._sample_time = value
        self.vehicle.update_sample_time(self._sample_time)
        self.view.dx = self._sample_time
    
    @property
    def t(self):
        return self._t



def main():
    
    sim = Simulator()
    sim.view.entry_point()

if __name__ == '__main__':
    main()



