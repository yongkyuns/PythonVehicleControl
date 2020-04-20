'''
.. module:: controller
   :synopsis: Control algorithms for lateral control of vehicle dynamics
.. moduleauthor:: Yongkyun Shin <github.com/yongkyuns>

This module is a collection of algorithms for control of lateral vehicle dynamics.
Currently implemented algorithms are PID controller, Model-Predictive controller,
and Deep-Deterministic Policy Gradient (DDPG) controller.
'''

import numpy as np
from scipy import sparse
import cvxpy as cp
import torch
from .learning.ddpg_agent import Agent
from .learning.model import Actor
import sys, os
# import acado

# DDPG Params
STATE_SIZE = 4
ACTION_SIZE = 1
RANDOM_SEED = 1
CHECK_POINT = 'checkpoint_actor.pth'

POS = 0
ANG = 1
NOW = 0
FUTURE = 4


class Controller:
    '''
    Generic controller parent class with some common variables (e.g. sample time, number of states, etc.)
    '''

    def __init__(self, NU=1, NY=1, T=1, dt=0.01):
        self.reference = np.zeros((NY, T))
        self.measured_output = np.zeros((NY, T))
        self.control_input = np.zeros(NU)
        self.dt = dt  # Controller sample time
        self.T = T   # Number of control points along the path

    def control(self):
        pass


class PathController(Controller):
    '''
    PathController has projection horizon in extension to generic Controller class.
    Projection horizon defines the preview, or lookahead distances w.r.t. vehicle
    '''
    def __init__(self, NU=1, NY=1, T=1, dt=0.01, proj_horizon=0.5):
        super().__init__(NU=NU, NY=NY, T=T, dt=dt)
        # [sec] Use T control points along desired path for proj_horizon time
        self.proj_horizon = proj_horizon

    @property
    def proj_step(self):
        return self.proj_horizon / self.T


class DDPG(PathController):
    '''
    Experimental controller which uses reinforcement learning algorithm.
    '''
    def __init__(self, check_point=CHECK_POINT, dt=0.01, T=1, NY=1, proj_horizon=0.5):
        super().__init__(dt=dt, T=T, proj_horizon=proj_horizon)
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        if torch.cuda.is_available():
            trained_model = torch.load(CHECK_POINT)
        else:
            trained_model = torch.load(
                CHECK_POINT, map_location={'cuda:0': 'cpu'})

        self.agent = Agent(state_size=STATE_SIZE,
                           action_size=ACTION_SIZE, random_seed=RANDOM_SEED)
        self.agent.actor_local = Actor(
            STATE_SIZE, ACTION_SIZE, RANDOM_SEED).to(device)
        self.agent.actor_local.load_state_dict(trained_model)

    def get_state(self, yref):
        # current/future pos reference, current/future heading reference
        states = np.array([yref[POS, NOW], yref[POS, FUTURE],
                           yref[ANG, NOW], yref[ANG, FUTURE]])
        states = np.reshape(states, [ACTION_SIZE, STATE_SIZE])
        return states

    def control(self, yref):
        states = self.get_state(yref)
        action = self.agent.act(states, add_noise=False)
        return float(action)


class PID(PathController):
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
                       dt=0.01,
                       P_Gain=1,
                       I_Gain=0.1,
                       D_Gain=0.01,
                       weight_split_T=None,
                       weight_split_Y=None,
                       proj_horizon=0.5):
        super().__init__(T=T, dt=dt, proj_horizon=proj_horizon)
        self.P_Gain = P_Gain
        self.I_Gain = I_Gain
        self.D_Gain = D_Gain

        self.weight_table = self.__calc_weight_table(
            T, NY, weight_split_T, weight_split_Y)

        self._integral = np.zeros(T)
        self._error_old = np.zeros(T)

    def __calc_weight_table(self, T, NY, T_split, Y_split):
        '''
        Update normalized weight table for PID control. 
        '''
        if T_split == None and Y_split == None:
            table = np.ones((NY, T)) / (NY*T)
        else:
            table = np.zeros((NY, T))
            vec_T = np.linspace(T_split[0], T_split[1], num=T)
            vec_Y = Y_split
            for i in range(NY):
                for j in range(T):
                    table[i, j] = vec_Y[i] * vec_T[j]
            if np.sum(table) > 0:
                table /= np.sum(table)
        return table

    def control(self, error_table):
        '''
        Calculate control input
        '''
        u = np.zeros(self.T)
        for i in range(self.T):
            e = 0
            for j in range(self.weight_table.shape[0]):
                e += self.weight_table[j, i] * error_table[j, i]
            self._integral[i] += e * self.dt
            u[i] = (self.P_Gain * e) + (self.I_Gain * self._integral[i]
                                        ) + (self.D_Gain * (e-self._error_old[i])/self.dt)
            self._error_old[i] = e
        return np.sum(u)


class MPC(PathController):
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

    def __init__(self, getModel,
                 T=5,
                 dt=0.01,
                 output_weight=None,
                 input_weight=None,
                 input_lim=None,
                 input_rate_lim=None,
                 proj_horizon=0.5):
        
        super().__init__(NU=1, NY=1, T=T, dt=dt, proj_horizon=proj_horizon)
        self.getModel = getModel
        self.output_weight = output_weight
        self.input_weight = input_weight
        self.input_lim = input_lim
        self.input_rate_lim = input_rate_lim

        self.setup_problem()
        

    def setup_problem(self):
        A, B, C, _ = self.getModel(sample_time=self.dt)
        NU = B.shape[1]  # Number of inputs to state-space model
        NX = A.shape[1]  # Number of states
        NY = C.shape[0]  # Number of outputs
        T = self.T  # Prediction horizon
        Q_DEFAULT = 5
        R_DEFAULT = 1

        # Constraints
        if self.input_lim is not None:
            umax = np.array(self.input_lim)
            umin = -1 * umax
        if self.input_rate_lim is not None:
            urmax = np.array([[1], [1]]) * 1
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
        self.u = cp.Variable((NU, T+1)) # T has one extra buffer element at the end 
        x = cp.Variable((NX, T+1))
        y = cp.Variable((NY, T+1))
        self.y_ref = cp.Parameter((NY, T))
        # self.y_ref.value = np.zeros((NY, T))
        self.x_init = cp.Parameter(NX)
        # self.x_init.value = np.zeros((NX,1))
        objective = 0
        constraints = [x[:, 0] == self.x_init]

        for k in range(T):
            objective += cp.quad_form(y[:, k] -
                                         self.y_ref[:, k], Q) + cp.quad_form(self.u[:, k], R)

            constraints += [x[:, k+1] == A*x[:, k] + B*self.u[:, k]]
            constraints += [y[:, k] == C*x[:, k]]

            if self.input_rate_lim is not None:
                constraints += [urmin <= self.u[:, k+1] -
                                self.u[:, k], self.u[:, k+1] - self.u[:, k] <= urmax]
            if self.input_lim is not None:
                constraints += [umin <= self.u[:, k], self.u[:, k] <= umax]

        self.prob = cp.Problem(cp.Minimize(objective), constraints)


    def update_values(self, x_init, y_ref):
        self.y_ref.value = y_ref
        self.x_init.value = x_init

    def control(self,y_desired, init_state):
        self.update_values(init_state, y_desired)
        # self.prob.solve(solver=cp.OSQP, max_iter=100,
                #    verbose=False, warm_start=True)
        self.prob.solve(cp.ECOS)
        return self.u[:, 0].value[0]

    def control_with_acado(self, y_desired, init_state):
        A, B, C, _ = self.getModel(sample_time=self.dt)
        NU = B.shape[1]    # Number of inputs to state-space model
        NX = A.shape[1]    # Number of states
        NYN = NX#C.shape[0]   # Number of outputs
        NY = NX+NU        # Control problem
        T = self.T         # Prediction horizon

        x0=np.zeros((1,NX))  
        X=np.zeros((T+1,NX))
        X[0:T,1] = y_desired[0,:]
        X[0:T,3] = y_desired[1,:]
        U=np.zeros((T,NU))    
        Y=np.concatenate((X[0:T,:],np.zeros((T,NU))),axis=1)
        # Y = np.zeros((T,NY))  
        yN=np.zeros((1,NYN))  
        x0[0,:]=init_state  # initial state 
        yN[0,:]=Y[-1,:NYN]         # reference terminal state 

        # Objective function
        Q_DEFAULT = 5
        R_DEFAULT = 1
        # Q = sparse.eye(NY) * Q_DEFAULT
        # Qf = sparse.eye(NYN) * Q_DEFAULT
        # R = sparse.eye(NU) * R_DEFAULT
        Q = np.diag([Q_DEFAULT,0,Q_DEFAULT,0,R_DEFAULT])
        Qf = np.diag([Q_DEFAULT,0,Q_DEFAULT,0])

        X, U = acado.mpc(0, 1, x0, X,U,Y,yN, np.transpose(np.tile(Q,T)), Qf, 0) 

        if U[0] != 0 or U[4] != 0:
            print('NOT ZERO!!!!')
        return 0

    # def control2(self, y_desired, init_state):
    #     A, B, C, _ = self.getModel(sample_time=self.dt)

    #     NU = B.shape[1]  # Number of inputs to state-space model
    #     NX = A.shape[1]  # Number of states
    #     NY = C.shape[0]  # Number of outputs
    #     T = self.T  # Prediction horizon
    #     Q_DEFAULT = 5
    #     R_DEFAULT = 1

    #     # Constraints
    #     if self.input_lim is not None:
    #         umax = np.array(self.input_lim)
    #         umin = -1 * umax
    #     if self.input_rate_lim is not None:
    #         urmax = np.array([[1], [1]]) * 1
    #         urmin = -1 * urmax

    #     # Objective function
    #     if self.output_weight is None:
    #         Q = sparse.eye(NY) * Q_DEFAULT
    #     else:
    #         Q = sparse.diags(self.output_weight)

    #     if self.input_weight is None:
    #         R = sparse.eye(NU) * R_DEFAULT
    #     else:
    #         R = sparse.diags(self.input_weight)

    #     # Define problem
    #     u = cp.Variable((NU, T+1)) # T has one extra buffer element at the end 
    #     x = cp.Variable((NX, T+1))
    #     y = cp.Variable((NY, T+1))
    #     yref = cp.Parameter((NY, T))
    #     x_init = cp.Parameter(NX)
    #     objective = 0
    #     constraints = [x[:, 0] == x_init]
        
    #     yref.value = y_desired
    #     x_init.value = init_state

    #     for k in range(T):
    #         objective += cp.quad_form(y[:, k] -
    #                                      yref[:, k], Q) + cp.quad_form(u[:, k], R)

    #         constraints += [x[:, k+1] == A*x[:, k] + B*u[:, k]]
    #         constraints += [y[:, k] == C*x[:, k]]

    #         if self.input_rate_lim is not None:
    #             constraints += [urmin <= u[:, k+1] -
    #                             u[:, k], u[:, k+1] - u[:, k] <= urmax]
    #         if self.input_lim is not None:
    #             constraints += [umin <= u[:, k], u[:, k] <= umax]

    #     prob = cp.Problem(cp.Minimize(objective), constraints)
    #     prob.solve(solver=cp.OSQP, max_iter=100,
    #                verbose=False, warm_start=True)

    #     return u[:, 0].value[0]



def main():
    pass


if __name__ == '__main__':
    main()
