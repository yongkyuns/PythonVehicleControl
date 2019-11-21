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
    def __init__(self,params,sample_time):
        A,B,C,D = self.__get_model_from_params(params,sample_time)
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self._sample_time = sample_time
        

    def update_model(self,params,sample_time=None):
        print('Updating dynamics model...')
        if sample_time is None:
            sample_time = self._sample_time

        A,B,C,D = self.__get_model_from_params(params,sample_time)
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self._sample_time = sample_time

    def get_sample_time(self):
        return self._sample_time

    def __get_model_from_params(self,params,sample_time):
        p = params
        Vx,m,Iz,lf,lr,Caf,Car = p.Vx, p.m, p.Iz, p.lf, p.lr, p.Caf, p.Car

        A = np.array([[           0,                  1,              0,                      0             ],
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
        self.model.update_model(self.params,sample_time=sample_time)
    
    def measure_output(self):
        return self.model.C @ self.states

    def __update_position(self):
        """
        Calculate updated global position [x, y] for current vehicle state
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
        "Moves" vehicle 1 time step forward with given steering angle input [rad]
        Returns (measured outputs, x position [m], y position [m] , heading angle [rad]) 
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
