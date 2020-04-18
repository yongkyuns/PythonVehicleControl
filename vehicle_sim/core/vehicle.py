"""
.. module:: vehicle
   :synopsis: Define classes required for defining vehicle 
              parameters and movement of the vehicle.
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module defines classes required for defining vehicle parameters 
and movement of the vehicle (dynamics).

Example:

.. code-block:: python

    import matplotlib.pyplot as plt
    import math
    import vehicle

    car = vehicle.Vehicle()
    N = 100
    output_hist = np.zeros((car.model.C.shape[0],N))
    state_hist = np.zeros((car.model.A.shape[0],N))
    x_hist = np.zeros((N,1))
    y_hist = np.zeros((N,1))
    yaw_hist = np.zeros((N,1))
    
    for i in range(N):
        state_hist[:,i] = car.states.flatten()
        output_hist[:,i],x_hist[i,0],y_hist[i,0],yaw_hist[i,0] = car.move(10*math.pi/180)
    
    plt.figure()
    plt.plot(state_hist[LAT_POS,:], label='lat. pos. [m]')
    plt.plot(state_hist[LAT_VEL,:], label='lat. vel. [m/s]')
    plt.plot(state_hist[YAW,:],     label='yaw [rad]')
    plt.plot(state_hist[YAW_RATE,:],label='yaw rate [rad/s]')

    plt.legend()
    plt.grid(True)
    plt.show()
"""

from scipy import signal
import numpy as np
import math

import sys
from vehicle_sim.helper.helper import pi_2_pi

from vehicle_sim.helper.logger import Logger
from vehicle_sim.core.controller import MPC, PID, DDPG
from vehicle_sim.core import path_planner
from vehicle_sim.core import car_params as cp

#states
LAT_POS  = 0
LAT_VEL  = 1
YAW      = 2
YAW_RATE = 3

NUM_WAYPTS = 6 # Number of waypoints to generate path from
PATH_LEN = 250 # [meters]
PATH_LATERAL_SIGMA = 10 # [meters] Sigma for normal distribution to vary lateral path points
PATH_VARIATION_FACTOR = 1 # [no unit] Used for scaling lateral waypoint variation. If larger, road gets more curvy toward end

class VehicleModel:
    '''
    This class implements basic vehicle dynamics model in discrete state-space form. 
    
    ================  ==================================================
    **Arguments:**
    params            (CarParams) Collection of parameters (e.g. mass, length, etc.) required for the bicycle model
    sample_time       (float) Sample time

    **Variables:**
    A                 (numpy array) System matrix
    B                 (numpy array) Input matrix
    C                 (numpy array) Output matrix
    D                 (numpy array) Initial condition
    ================  ==================================================
    '''
    def __init__(self,params,sample_time):
        self.A, self.B, self.C, self.D = self.get_model_from_params(params,sample_time)
        self._sample_time = sample_time

    def update_model(self,params,sample_time=None):
        '''
        Update the vehicle model based on updated params or sample time.

        ==================  ==================================================
        **Arguments:**
        params              (CarParams) Collection of parameters (e.g. mass, length, etc.) required for the bicycle model
        sample_time         (float) Sample time to be used for discretizing the state-space model.
        ==================  ==================================================
        '''
        print('Updating dynamics model...')
        if sample_time is None:
            sample_time = self._sample_time
        self.__init__(params, sample_time)

    def get_sample_time(self):
        '''
        Return the sample time used in the VehicleModel object.
        '''
        return self._sample_time

    def get_model_from_params(self,params,sample_time):
        '''
        Calculate and return the discretized state-space model.

        ==================  ==================================================
        **Arguments:**
        params              (CarParams) Collection of parameters (e.g. mass, length, etc.) required for bicycle model.
        sample_time         (float) Sample time to be used for discretizing the state-space model.

        **Returns:**
        A                   (numpy array) Discrete system matrix A
        B                   (numpy array) Discrete input matrix B
        C                   (numpy array) Discrete output matrix C
        D                   (numpy array) Initial condition D
        ==================  ==================================================
        '''
        p = params
        Vx,m,Iz,lf,lr,Caf,Car = p.Vx, p.m, p.Iz, p.lf, p.lr, p.Caf, p.Car

        A = np.array([[           0,                  1,              Vx,                      0             ],
                      [           0,       -(2*Caf + 2*Car)/(m*Vx),   0,   -Vx-(2*Caf*lf-2*Car*lr)/(m*Vx)   ],
                      [           0,                  0,              0,                      1             ],
                      [           0,    -(2*lf*Caf-2*lr*Car)/(Iz*Vx), 0,  -(2*lf**2*Caf+2*lr**2*Car)/(Iz*Vx)]])

        B = np.array([[     0     ],
                      [  2*Caf/m  ],
                      [     0     ],
                      [2*lf*Caf/Iz]])

        C = np.array([[1, 0, 0, 0], # lateral position
                      [0, 0, 1, 0]]) # yaw angle

        D = np.array(0)
        Ad,Bd,Cd,Dd,_ = signal.cont2discrete((A,B,C,D),sample_time)
        return Ad,Bd,Cd,Dd



class Vehicle:
    '''
    This class defines vehicle motion and related functions for simulating the movement of a vehicle.

    ====================  ==================================================
    **Arguments:**
    Vx                    (float) Vehicle speed [m/s]
    m                     (float) Vehicle mass [kg]
    Iz                    (float) Moment of inertia along z-axis [kg*m^2] 
    lf                    (float) Distance to the front axle from CG [m]
    lr                    (float) Distance to the rear axle from CG [m]
    Caf                   (float) Front tire cornering stiffness [N/rad]
    Car                   (float) Rear tire cornering stiffness [N/rad]
    sample_time           (float) Sample time for discretizing the dynamics model 
    init_state            ([float, float, float, float]) Initial state of the vehicle (lateral position[m], lateral velocity[m/s], yaw angle[rad], yaw rate[rad/s])
    
    **Variables:**
    states                ([float,float,float,float]) Current states (lateral position[m], lateral velocity[m/s], yaw angle[rad], yaw rate[rad/s])
    x                     (float) Global x position of vehicle [m]
    y                     (float) Global y position of vehicle [m]
    model                 (VehicleModel) Vehicle dynamics model
    measured              ([float, float]) Measured quantities (lateral velocity[m/s], yaw rate[rad/s])
    ====================  ==================================================
    '''
    def __init__(self, get_time_func=None,Vx=20,m=1600,Iz=1705,lf=1.4,lr=1.4,Caf=66243,Car=66243,sample_time=0.01,init_state=None,controller='PID'):
        self._params = cp.CarParams(Vx,m,Iz,lf,lr,Caf,Car)
        self.model = VehicleModel(self.params,sample_time)
        self.params.register_callback(self.model.update_model)

        if init_state is None:
            init_state = np.zeros((self.model.A.shape[1],1))
        self.init_state = init_state
        self.get_time_func = get_time_func

        self.reset()
        self.generate_new_path()

        if controller is 'PID':
            self.controller = PID(dt=sample_time,
                                  T=5,
                                  NY=2,
                                  P_Gain=1,
                                  I_Gain=0,
                                  D_Gain=0,
                                  weight_split_T=[0.5,0.5], 
                                  weight_split_Y=[0.5,0.5],
                                  proj_horizon=0.5)
        elif controller is 'MPC':
            self.controller = MPC(self.get_dynamics_model,
                                  T=5,
                                  dt=sample_time,
                                  proj_horizon=0.5)
        elif controller is 'DDPG':
            self.controller = DDPG(T=5,
                                   proj_horizon=0.5,
                                   dt=sample_time)
        else:
            print('Invalid controller option specified!! Either PID or MPC is accepted.')
    
    @property
    def params(self):
        return self._params
    @params.setter
    def params(self,value):
        self._params = value
        self._model.update_model(self._params)
    
    @property
    def model(self):
        return self._model
    @model.setter
    def model(self,value):
        self._model = value

    @property
    def yaw(self):
        return pi_2_pi(self.states[YAW].item())

    def reset(self):
        self.states = self.init_state
        self.measured = self.measure_output()
        self.x = 0
        self.y = 0
        self.logger = Logger()

    def update_sample_time(self,sample_time):
        '''
        Change the sample time and update the dynamics model.
        '''
        self.model.update_model(self.params,sample_time=sample_time)
    
    def measure_output(self):
        '''
        Return the measurement values of the output states.
        '''
        output = self.model.C @ self.states
        return output.flatten()

    def generate_new_path(self):
        x,y = self.__generate_random_waypoints()
        self.path = path_planner.Planner(x,y)
        # return path_planner.Planner(x,y)

    def __generate_random_waypoints(self):
        # Global path waypoints
        # x = [0, 50, 100, 150, 200, 250]
        # y = [0, 0, 30, 60, 60, 0]
    
        N = NUM_WAYPTS
        x = np.linspace(0, PATH_LEN, N)
        y = [0] * N
        mu, sigma = 0, PATH_LATERAL_SIGMA

        s = np.random.normal(mu, sigma, N)
        for i in range(1,N-1):
            y[i+1] = y[i] + s[i] * (1+(i+1)/N*PATH_VARIATION_FACTOR) 

        return list(x),list(y)

    def get_dynamics_model(self,sample_time=None):
        '''
        Returns the discrete state-space dynamics model. If sample_time is specified, returns 
        discretized model which is based on specified sample time (this may not correspond
        to the actual state-space model of the current Vehicle object). This function is intended
        to be used for implementing model-based control, such as MPC.

        ==================  ==================================================
        **Returns:**
        A                   (numpy array) Discrete ystem matrix A
        B                   (numpy array) Discrete input matrix B
        C                   (numpy array) Discrete output matrix C
        D                   (numpy array) Initial condition D
        ==================  ==================================================
        '''
        if sample_time == None:
            A,B,C,D = self.model.A, self.model.B, self.model.C, self.model.D
        else:
            A,B,C,D = self.model.get_model_from_params(self.params,sample_time)
        return A,B,C,D

    def __update_position(self):
        """
        Perform 1 time step update of the global position [x, y] based on the current vehicle states.
        """
        yaw = self.states[YAW]
        Vx = self.params.Vx
        Vy = self.states[LAT_VEL]
        V = math.sqrt(Vx**2 + Vy**2)
        ang = yaw + math.tan(Vy/Vx)
        dt = self.model.get_sample_time()
        self.x += V*math.cos(ang)*dt
        self.y += V*math.sin(ang)*dt

    def move(self,str_ang=None):
        """
        "Moves" vehicle 1 time step forward with given steering angle input [rad]. Use this function to execute 1 time step of simulation with internal dynamics model.

        ==================  ==================================================
        **Arguments:**
        steering_ang        (float) Wheel steering angle [rad]. Positive angle is right side.

        **Returns:**
        Measured Outputs    ([float, float]) Lateral velocity [m/s] and yaw rate [rad/s]
        x                   (float) Global x position of the vehicle [m]
        y                   (float) Global y position of the vehicle [m]
        yaw                 (float) Heading angle of the vehicle [rad]
        ==================  ==================================================
        """

        if str_ang is None:
            ctrl_pt_x, ctrl_pt_y, path_x, path_y, yref = self.detect_and_plan()
            str_ang = self.control(yref)
        else:
            ctrl_pt_x, ctrl_pt_y, path_x, path_y = 0,0,0,0
            yref = np.array([[0,0,0,0,0],[0,0,0,0,0]])

        steering_ang = np.array(str_ang).reshape(1,1)
        self.measured = self.measure_output()
        self.states = self.model.A @ self.states + self.model.B @ steering_ang
        # self.states = self.model.A.dot(self.states) + self.model.B.dot(np.array([[steering_ang]]))
        self.__update_position()

        
        self.logger.log(self.get_time_now(), ctrl_pt_x, ctrl_pt_y, path_x, path_y, self.measured, yref[0,4], self.x, self.y, self.yaw, str_ang)

        return self.measured.flatten(), self.x, self.y, self.yaw

    def get_time_now(self):
        if self.get_time_func is not None:
            t = self.get_time_func()
        else:
            # print('Time function is not available in Vehicle object. Returning 0')
            t = 0
        return t

    def control(self,yref):
        '''
        Determine steering input
        '''
        if isinstance(self.controller,MPC):
            state = np.array([0,self.states[LAT_VEL].item(),0,self.states[YAW_RATE].item()])
            str_ang = self.controller.control(yref,state)
            # str_ang = self.controller.control_with_acado(yref,state)
            
        elif isinstance(self.controller, PID) or isinstance(self.controller, DDPG):
            str_ang = self.controller.control(yref)
        else:
            print('Incorrect controller class being used!')
        return str_ang

    def detect_and_plan(self):
        # Update vehicle states
        Vx = self.params.Vx
        
        # Generate local path to follow --> subset of global path near vehicle in relative coordinate frame
        # rel_path_x, rel_path_y, rel_path_yaw = self.path.detect_local_path(pos_x,pos_y,yaw)
        rel_path_x, rel_path_y, rel_path_yaw = self.path.detect_local_path_with_FOV(self.x,self.y,self.yaw)
        step = Vx*self.controller.proj_step

        # Calculate reference output (lateral position, heading) from desired path
        x_ref, pos_ref, yaw_ref = self.path.calc_ref(rel_path_x, rel_path_y, rel_path_yaw, step, self.controller.T)
        yref = np.array([pos_ref,yaw_ref])
        return x_ref, pos_ref, rel_path_x, rel_path_y, yref


def main():
    car = Vehicle()
    N = 100
    for i in range(N):
        car.move()

        

if __name__ == '__main__':
    import cProfile
    cProfile.run('main()',sort='cumtime')

    # import matplotlib.pyplot as plt

    # N = 11
    # ax = np.linspace(0, 500, N)
    # ay = [0] * N
    # mu, sigma = 0, 5

    # for j in range(20):
    #     s = np.random.normal(mu, sigma, N)
    #     for i in range(N-1):
    #         # ay[i+1] = ay[i] + np.random.randint(-30,30)
    #         ay[i+1] = ay[i] + s[i] * (1+(i+1)/N) 
    #     path = path_planner.Planner(ax,ay)
    #     plt.plot(path.x,path.y)

    # plt.axis('equal')
    # plt.grid()
    # plt.show()