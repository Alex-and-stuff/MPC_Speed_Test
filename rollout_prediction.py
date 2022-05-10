import numpy as np
from sympy import im
from parameters import *
from bicycle_model_mppi import *
from state_cost import *
import time

def ClampingFcn(u):
    v = u[0]
    w = u[1]
    if v > VD_MAX: v = VD_MAX
    if v < VD_MIN: v = VD_MIN
    if w > WD_MAX: w = WD_MAX
    if w < WD_MIN: w = WD_MIN
    return np.array([v,w])

def RolloutPrediction(u, x0, inn_fcn, out_fcn, obstacles):
    hrz = np.zeros((STATES, N_HRZ))
    s = 0
    u_pert = np.random.normal(PERT_U, PERT_STDEV, size = (CONTROLS, N_HRZ-1))
    u_n = u + u_pert
    hrz[:,0] = x0
    
    for nth_step in range(N_HRZ-1):
        
        u_n[:, nth_step] = ClampingFcn(u_n[:,nth_step])
        x_dot = BicycleModelMPPI(hrz[:,nth_step], u_n[:, nth_step])
        hrz[:,nth_step+1] = hrz[:,nth_step] + x_dot*T_STP
        s += StateCost(hrz[:,nth_step], x_dot, u_n[:, nth_step], inn_fcn, out_fcn, obstacles)   

    return hrz, s, u_n



# u  = np.zeros((CONTROLS, N_HRZ-1))
# x0 = np.array([1,2,3,4])
# for k in range(N_HRZ-1):
#     u[:,k] = np.array([2,0])
# RolloutPrediction(u, x0)