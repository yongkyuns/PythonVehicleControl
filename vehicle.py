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
from helper import *

sys.path.append('.')
import car_params as cp

#states
LAT_POS  = 0
LAT_VEL  = 1
YAW      = 2
YAW_RATE = 3

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
    def __init__(self,Vx=20,m=1600,Iz=1705,lf=1.4,lr=1.4,Caf=66243,Car=66243,sample_time=0.01,init_state=None):
        self._params = cp.CarParams(Vx,m,Iz,lf,lr,Caf,Car)
        self.model = VehicleModel(self.params,sample_time)
        self.params.register_callback(self.model.update_model)

        if init_state is None:
            init_state = np.zeros((self.model.A.shape[1],1))
        self.states = init_state
        self.measured = self.measure_output()
        self.x = 0
        self.y = 0

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

    def update_sample_time(self,sample_time):
        '''
        Change the sample time and update the dynamics model.
        '''
        self.model.update_model(self.params,sample_time=sample_time)
    
    def measure_output(self):
        '''
        Return the measurement values of the output states.
        '''
        return self.model.C @ self.states

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

    def move(self,steering_ang):
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
        steering_ang = np.array(steering_ang).reshape(1,1)
        self.measured = self.measure_output()
        self.states = self.model.A @ self.states + self.model.B @ steering_ang
        # self.states = self.model.A.dot(self.states) + self.model.B.dot(np.array([[steering_ang]]))
        self.__update_position()
        return self.measured.flatten(), self.x, self.y, self.yaw

def main():
    import matplotlib.pyplot as plt
    import math

    car = Vehicle()
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
        

if __name__ == '__main__':
    main()
