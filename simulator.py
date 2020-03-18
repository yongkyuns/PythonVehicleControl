'''
.. module:: simulator
   :synopsis: Simulator class handles execution of simulation 
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module defines classes needed for executing simulation. All of the simulation-related aspects
(e.g. simulation step, updating graphics, executing and coordinating object movement) are done
by the simulator module.
'''

import vehicle
import numpy as np
import time
import sys

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
    dt                (float) sample time of the simulation. Changing sample time will update discrete dynamics of member objects.
    sim_time          (float) simulation time in seconds
    online_mode       (bool)  Specify visualization mode. If True, update graphics along with simualtion at runtime.

    **Variables:**
    currentStep       (int) Current simulation step
    view              (Visualizer) Main view for display              
    ================  ==================================================
    '''

    def __init__(self, dt=0.01, sim_time=1, online_mode=False, verbose=False):
        self.verbose = verbose
        self._dt = dt # [sec]
        self.vehicle = vehicle.Vehicle(get_time_func=self.get_time,sample_time=self.sample_time,controller='PID')
        self.sim_time = sim_time
        self.init_param()
    
    def init_param(self):
        self.currentStep = 0
        self._t = 0
        self._prev_progress = 0

    def __update_progress(self, msg):
        progress = np.int(self.t / self.sim_time * 100)
        if progress is not self._prev_progress:
            if msg is not None:
                msg.send(progress)
            self._prev_progress = progress        

    def run(self,msg=None):
        '''
        Invoke entry_point of the Visualizer
        '''
        time_begin = time.time()
        if self.verbose is True:
            print('Starting simulation...')
        self.vehicle.reset()
        while self._t <= self.sim_time:
            self.step()
            self.__update_progress(msg)
        
        self.init_param()

        if self.verbose is True:
            time_end = time.time()
            print('Finished running simulation!!!')
            print('Time to run ' + ('%.1f' %self.sim_time) + ' seconds of simulation = ' + ('%.3f' %(time_end-time_begin)) + ' seconds!!')

        return self.vehicle.logger

    def step(self):
        '''
        Execute 1 time step of simulation.
        '''

        if self.verbose is True:
            print('t = ' + ('%.3f' %self.t) + ' ms')

        self.vehicle.move()

        self._t += self._dt
        self.currentStep += 1

    def get_time(self):
        return self.t

    @property
    def dt(self):
        return self._dt
    @dt.setter
    def sample_time(self,value):
        self._dt = value
        self.vehicle.update_sample_time(self._dt)
        # self.view.dx = self._dt
    
    @property
    def t(self):
        return self._t

def call_back(progress):
    print(progress)

def main():
    begin = time.time()
    sim = Simulator(sim_time=20,verbose=False)
    log = sim.run()
    print(time.time() - begin)
    print('Final x position = ' + str(log.x[-1]) + ' m ')


if __name__ == '__main__':
    main()



