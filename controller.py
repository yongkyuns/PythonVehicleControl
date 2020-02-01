'''
.. module:: controller
   :synopsis: Control algorithms for lateral control of vehicle dynamics
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module is a collection of algorithms for control of lateral vehicle dynamics.
'''

import numpy as np
from scipy import sparse
import cvxpy

class Controller:
    '''
    Generic controller parent class with some common variables (e.g. sample time, number of states, etc.)
    '''
    def __init__(self,NU=1,NY=1,T=1,dt=0.1):
        self.reference = np.zeros((NY,T))
        self.measured_output = np.zeros((NY,T))
        self.control_input = np.zeros(NU)
        self.dt = dt
        self.T = T
    
    def control(self):
        pass

class PID(Controller):
    '''
    PID controller with variable number of control points along desired path.
    Control parameters are lateral position and heading. For each control point & parameter,
    there is a corresponding normalized weight which adds up to 1. Therefore, a single PID
    controller is used to control all of the controlled states and points along desired path.
    
    ================  ==================================================
    **Arguments:**
    NU                (int) Number of controlled states
    NY                (int) Number of outputs
    T                 (int) Number of reference points along trajectory
    dt                (float) Sample time [sec] of the controller
    P_Gain            (float) Proportional gain of the controller
    I_Gain            (float) Integral gain of the controller
    D_Gain            (float) Derivative gain of the controller
    weight_split_T    (list) Determine how to split weight on reference points (e.g. [0.5, 0.5] for equal split on all reference points)
    weight_split_Y    (list) Determine how to split weight on measured output (e.g. [0.5, 0.5] for equal split on position and heading control)
    
    **Variables:**
    weight_table      (numpy array) NY-by-T array of normalized weight for each controlled point & parameter 
    ================  ==================================================
    '''
    def __init__(self, NU=1,
                       NY=1,
                       T=1,
                       dt=0.1,
                       P_Gain=1,
                       I_Gain=0.1,
                       D_Gain=0.01,
                       weight_split_T=None,
                       weight_split_Y=None):
        super().__init__(NU=NU,NY=NY,T=T,dt=dt)
        self.P_Gain = P_Gain
        self.I_Gain = I_Gain
        self.D_Gain = D_Gain

        self.weight_table = self.__calc_weight_table(T,NY,weight_split_T,weight_split_Y) 

        self._integral = np.zeros(T)
        self._error_old = np.zeros(T)

    def __calc_weight_table(self, T, NY, T_split, Y_split):
        '''
        Update normalized weight table for PID control. 
        '''
        if T_split == None and Y_split == None:
            table = np.ones((NY,T)) / (NY*T)
        else:
            table = np.zeros((NY,T))
            vec_T = np.linspace(T_split[0], T_split[1], num=T)
            vec_Y = Y_split
            for i in range(NY):
                for j in range(T):
                    table[i,j] =  vec_Y[i] * vec_T[j] 
            if np.sum(table) > 0:
                table /= np.sum(table)
        return table

    def control(self,error_table):
        '''
        Calculate control input
        '''
        u = np.zeros(self.T)
        for i in range(self.T):
            e = 0
            for j in range(self.weight_table.shape[0]):
                e += self.weight_table[j,i] * error_table[j,i]
            self._integral[i] += e * self.dt
            u[i] = (self.P_Gain * e) + (self.I_Gain * self._integral[i]) + (self.D_Gain * (e-self._error_old[i])/self.dt) 
            self._error_old[i] = e
        return np.sum(u)

class MPC(Controller):
    '''
    Generic model predictive controller class. Uses linearized state-space model.
    Convex optimization is solved through cvxpy module.
    
    ================  ==================================================
    **Arguments:**
    getModel          (function) Function which should return linearized state-space dynamics model as numpy array
    NU                (int) Number of controlled states
    NY                (int) Number of outputs
    T                 (int) Number of reference points along trajectory
    dt                (float) Sample time [sec] of the controller
    output_weight     (float) Weight for control of the output
    input_weight      (float) Weight for input constraint control
    input_lim         (float) Control input limit
    input_rate_lim    (float) Control input rate limit
    ================  ==================================================
    '''
    def __init__(self,getModel,
                          NU=1,
                          NY=1,
                          T=5,
                          dt=0.1,
                          output_weight=None,
                          input_weight=None,
                          input_lim=None,
                          input_rate_lim=None):
        super().__init__(NU=NU,NY=NY,T=T,dt=dt)
        self.getModel = getModel
        self.output_weight = output_weight
        self.input_weight = input_weight
        self.input_lim = input_lim
        self.input_rate_lim = input_rate_lim

    def control(self,yref,init_state):
        A,B,C,_ = self.getModel(sample_time=self.dt)

        NU = B.shape[1] # Number of inputs to state-space model
        NX = A.shape[1] # Number of states
        NY = C.shape[0] # Number of outputs
        T = self.T  # Prediction horizon
        Q_DEFAULT = 5
        R_DEFAULT = 1
        
        # Constraints
        if self.input_lim is not None:
            umax = np.array(self.input_lim)
            umin = -1 * umax
        if self.input_rate_lim is not None:
            urmax = np.array([[1],[1]]) * 1
            urmin = -1 * urmax

        # Objective function
        if self.output_weight is None:
            Q = sparse.eye(NY) * Q_DEFAULT
        else:
            Q = sparse.diags(self.output_weight)

        if self.input_weight is None:
            R = sparse.eye(NU) * R_DEFAULT
        else:
            R = sparse.diags(self.input_weight)

        # Define problem
        u = cvxpy.Variable((NU, T+1))
        x = cvxpy.Variable((NX, T+1))
        y = cvxpy.Variable((NY, T+1))
        x_init = cvxpy.Parameter(NX)
        x_init.value = init_state
        objective = 0
        constraints = [x[:,0] == x_init]

        for k in range(T):
            objective += cvxpy.quad_form(y[:,k] - yref[:,k], Q) + cvxpy.quad_form(u[:,k], R)

            constraints += [x[:,k+1] == A*x[:,k] + B*u[:,k]]
            constraints += [y[:,k] == C*x[:,k]]

            if self.input_rate_lim is not None:
                constraints += [urmin <= u[:,k+1] - u[:,k], u[:,k+1] - u[:,k] <= urmax]
            if self.input_lim is not None:
                constraints += [umin <= u[:,k], u[:,k] <= umax]

        prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)
        prob.solve(solver=cvxpy.OSQP, max_iter=100, verbose=False, warm_start=True)

        return u[:,0].value[0]




def main():
    pass

if __name__ == '__main__':
    main()
