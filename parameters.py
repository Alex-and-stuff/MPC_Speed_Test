import numpy as np

STATES   = 4
CONTROLS = 2
T_HRZ    = 2
F_STP    = 20
T_STP    = 1/F_STP
N_HRZ    = T_HRZ*F_STP

VD_MAX  = 4
VD_MIN  = 1.5
WD_MAX  = 1.5
WD_MIN  = -1.5

DT      = 0.05
T_END   = 5
N       = int(T_END/DT)
K       = 10000

PERT_STDEV = 2
PERT_U     = 0
LAMBDA     = 50

''' Build obstacles '''
obstacle = np.array([[154 ,125.7 ,1],
                     [152 ,119   ,1]])

''' Build track ''' 
mid_fcn = np.array([[1.0000,   -1.33200,    82.62978],
                    [1.0000,    0.75640,  -240.86623],
                    [1.0000,   -1.36070,   -33.13473],
                    [1.0000,    0.47203,   -35.00739]])
       
ROAD_WIDTH = 2

OFF_ROAD_P = 500
COLLISION_P = 800
TRACK_ERR_P = 5
VD_P        = 10
WD_P        = 10


