import math
import numpy as np

def BicycleModelMPPI(state, control):
    x = state[0]
    y = state[1]
    phi = state[2]
    v0  = state[3]
    Ka = 2.54
    vd = control[0]
    wd = control[1]

    x_dot = v0*math.cos(phi)
    y_dot = v0*math.sin(phi)
    phi_dot = wd
    v_dot = Ka*v0*(vd-v0)

    state_dot = np.array([x_dot, y_dot, phi_dot, v_dot])

    return state_dot