'''
.. module:: simulator
   :synopsis: Top-level simulator class for executing simulation
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module defines classes needed for executing simulation. All of the simulation-related aspects
(e.g. simulation step, updating graphics, executing and coordinating object movement) are done
by the simulator module.
'''

from controller import MPC, PID
import vehicle
import path_planner
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
        self.vehicle = vehicle.Vehicle(sample_time=self.sample_time)
        
        self.N = 10000
        self.currentStep = 0
        self._t = 0

        ax = [0, 50, 100, 150, 200, 250]
        ay = [0, 0, 30, 60, 60, 60]

        self.view = view.Visualizer(self.step,dx=self.sample_time)
        self.path = path_planner.Planner(ax,ay)
        self.update_view_data(self.path.x,self.path.y,self.view.graph['global_path'])

        self.controller = PID(T=5,NY=2,P_Gain=1,I_Gain=0,D_Gain=0,weight_split_T=[0.5,0.5], weight_split_Y=[0.5,0.5])
        # self.controller = MPC(self.vehicle.get_dynamics_model)

        self.init_log()

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

    def init_log(self):
        N = self.N
        self.output_hist = []
        self.output_ref_hist = []
        self.x_hist = []
        self.y_hist = []
        self.yaw_hist = []

        self.t_hist = []
        self.str_ang_hist = []

    def run(self):
        '''
        Invoke entry_point of the Visualizer
        '''
        self.view.entry_point()

    def calcSteeringAng(self):
        '''
        Determine steering input
        '''
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
        self.update_view_data(x_ref,pos_ref,self.view.graph['ctrl_pts'])
        self.update_view_data(rel_path_x,rel_path_y,self.view.graph['local_path'])

        return yref, str_ang

    def step(self):
        '''
        Execute 1 time step of simulation.
        '''
        i = self.currentStep
        self._t += self._sample_time

        yref, str_ang = self.calcSteeringAng()

        output, x, y, yaw = self.vehicle.move(str_ang)

        self.output_hist.append(output)
        self.x_hist.append(x)
        self.y_hist.append(y)
        self.yaw_hist.append(yaw)
        self.output_ref_hist.append(yref[0,4])
        self.t_hist.append(self.t)
        self.str_ang_hist.append(str_ang)

        self.view.graph['car'].setData(x=x,y=y,z_ang=yaw*180/np.pi)
        self.view.graph['str_ang'].setData(x=self.t_hist,y=self.str_ang_hist)
        self.view.graph['ref1_err'].setData(x=self.t_hist,y=self.output_ref_hist)

        
        # xRange = 2
        # xPos = self.t_hist[-1]
        # xMin = max(0, xPos-xRange/2)
        # xMax = max(1,xPos+xRange/2)

        # # self.view.graph['str_ang'].setRange(xRange=[xMin,xMax])
        # # self.view.graph['str_ang'].setPos(self.t_hist[-1],0)
        # self.view.graph['str_ang'].setLimits(xMin=-0.5,xMax=self.t_hist[-1]+0.5)
        

        self.currentStep += 1


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



