```python
"""
Contains controllers a.k.a. agents.
"""

from utilities import dss_sim
from utilities import rep_mat
from utilities import uptria2vec
from utilities import push_vec
import models
import numpy as np
import scipy as sp
from numpy.random import rand
from scipy.optimize import minimize
from scipy.optimize import basinhopping
from scipy.optimize import NonlinearConstraint
from scipy.stats import multivariate_normal
from scipy.linalg import solve_discrete_are
from numpy.linalg import lstsq
from numpy import reshape
import warnings
import math
from tabulate import tabulate
import os

def kinematic_controller(x, x_ref, gains):
    dx = x_ref[0] - x[0]
    dy = x_ref[1] - x[1]
    theta = x[2]
    rho = np.sqrt(dx**2 + dy**2)
    alpha = np.arctan2(dy, dx) - theta
    beta = x_ref[2] - np.arctan2(dy, dx)
    alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
    beta = np.arctan2(np.sin(beta), np.cos(beta))
    k_rho, k_alpha, k_beta = gains
    v = k_rho * rho
    omega = k_alpha * alpha + k_beta * beta
    return np.array([v, omega])


def ctrl_selector(t, observation, action_manual, ctrl_nominal, ctrl_benchmarking, mode):
    if mode == 'manual':
        return action_manual
    elif mode == 'nominal':
        return ctrl_nominal.compute_action(t, observation)
    else:
        return ctrl_benchmarking.compute_action(t, observation)


class LQR:
    def __init__(self, A, B, Q, R, observation_target, sampling_time=0.1):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.observation_target = observation_target
        self.K = self.compute_gain()
        self.sampling_time = sampling_time
        self.ctrl_clock = 0.0
        self.action_curr = np.zeros(B.shape[1])

    def compute_gain(self):
        P = solve_discrete_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R + self.B.T @ P @ self.B) @ (self.B.T @ P @ self.A)
        return K

    def compute_action(self, t, observation):
        if t >= self.ctrl_clock + self.sampling_time - 1e-6:
            self.ctrl_clock = t
            error = observation - self.observation_target
            u = -self.K @ error
            self.action_curr = u
        return self.action_curr

class ControllerOptimalPredictive:
    def __init__(self, dim_input, dim_output, Nactor, sampling_time, pred_step_size, sys_rhs, sys_out, ctrl_bnds, run_obj_pars, observation_target):
        self.dim_input = dim_input
        self.dim_output = dim_output
        self.Nactor = Nactor
        self.sampling_time = sampling_time
        self.pred_step_size = pred_step_size
        self.sys_rhs = sys_rhs
        self.sys_out = sys_out
        self.observation_target = observation_target
        self.run_obj_pars = run_obj_pars
        self.ctrl_clock = 0.0

        self.action_min = np.array(ctrl_bnds[:, 0])
        self.action_max = np.array(ctrl_bnds[:, 1])
        self.action_curr = np.zeros(self.dim_input)

        self.action_sqn_min = rep_mat(self.action_min, 1, Nactor)
        self.action_sqn_max = rep_mat(self.action_max, 1, Nactor)
        self.action_sqn_init = rep_mat((self.action_min + self.action_max) / 2, 1, Nactor)

    def run_obj(self, observation, action):
        R1 = self.run_obj_pars[0]
        chi = np.concatenate([observation, action])
        return chi.T @ R1 @ chi

    def _actor_cost(self, action_sqn, observation):
        my_action_sqn = np.reshape(action_sqn, [self.Nactor, self.dim_input])
        observation_sqn = np.zeros([self.Nactor, self.dim_output])
        observation_sqn[0, :] = observation
        state = self.sys_out(observation)
        for k in range(1, self.Nactor):
            state = state + self.pred_step_size * self.sys_rhs([], state, my_action_sqn[k-1, :])
            observation_sqn[k, :] = self.sys_out(state)
        J = 0
        for k in range(self.Nactor):
            J += self.run_obj(observation_sqn[k, :], my_action_sqn[k, :])
        return J

    def _actor_optimizer(self, observation):
        my_action_sqn_init = np.reshape(self.action_sqn_init, [self.Nactor*self.dim_input, ])
        bnds = sp.optimize.Bounds(self.action_sqn_min, self.action_sqn_max, keep_feasible=True)
        try:
            result = minimize(lambda a: self._actor_cost(a, observation), my_action_sqn_init, method='SLSQP', bounds=bnds)
            action_sqn = result.x
        except ValueError:
            print('MPC optimizer failed. Returning last known action.')
            action_sqn = np.tile(self.action_curr, self.Nactor)
        return action_sqn[:self.dim_input]

    def compute_action(self, t, observation):
        if t >= self.ctrl_clock + self.sampling_time - 1e-6:
            self.ctrl_clock = t
            action = self._actor_optimizer(observation)
            self.action_curr = action
        return self.action_curr
```
