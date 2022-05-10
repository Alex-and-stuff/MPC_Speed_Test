from os import stat
import numpy as np
from parameters import *
import math

def DistanceFromTrack(inn_f, inn_g, inn_h, inn_i, inn_fcn):
    slope = -inn_fcn[:,1]
    distance = 0
    if inn_f<0 and inn_g<0 and inn_h<0 and inn_i>0:
        distance = abs(abs(inn_f) - 0.5*ROAD_WIDTH/math.cos(math.atan(slope[0])))

    elif inn_f<0 and inn_g>0 and inn_h<0 and inn_i>0:
        distance = abs(math.sqrt(math.pow(inn_f,2)+math.pow(inn_g,2)) - 0.5*ROAD_WIDTH)
    
    elif inn_f>0 and inn_g>0 and inn_h<0 and inn_i>0:
        distance = abs(abs(inn_g) - 0.5*ROAD_WIDTH/math.cos(math.atan(slope[1])))
        
    elif inn_f>0 and inn_g>0 and inn_h>0 and inn_i>0:
        distance = abs(math.sqrt(math.pow(inn_g,2)+math.pow(inn_h,2)) - 0.5*ROAD_WIDTH)
        
    elif inn_f>0 and inn_g<0 and inn_h>0 and inn_i>0:
        distance = abs(abs(inn_h) - 0.5*ROAD_WIDTH/math.cos(math.atan(slope[2])))
        
    elif inn_f>0 and inn_g<0 and inn_h>0 and inn_i<0:
        distance = abs(math.sqrt(math.pow(inn_h,2)+math.pow(inn_i,2)) - 0.5*ROAD_WIDTH)
        
    elif inn_f>0 and inn_g<0 and inn_h<0 and inn_i<0:
        distance = abs(abs(inn_i) - 0.5*ROAD_WIDTH/math.cos(math.atan(slope[3])))
        
    elif inn_f>0 and inn_g<0 and inn_h<0 and inn_i<0:
        distance = abs(math.sqrt(math.pow(inn_i,2)+math.pow(inn_f,2)) - 0.5*ROAD_WIDTH)
        
    elif inn_f==0 or inn_g==0 or inn_h==0 or inn_i==0:
        inn_dis = [inn_f, inn_g, inn_h, inn_i]
        min_d = abs(inn_dis) 
        distance = min(min_d(min_d>0)) - 0.5*ROAD_WIDTH; 
    else:
        a = 0 # do nothing

    if distance > ROAD_WIDTH/2:
        distance = 2

    return distance

def ObstacleCollision(state, pos_and_size):
    x = state[0]
    y = state[1]
    phi = state[2]

    n_obstacles, _ = pos_and_size.shape
    result = np.zeros((n_obstacles, 3))

    for nth_obstacle in range(n_obstacles):
        obstacle_fcn = math.pow(x-pos_and_size[nth_obstacle,0],2)+math.pow(y-pos_and_size[nth_obstacle,1],2)-math.pow(pos_and_size[nth_obstacle, 2],2)
        if obstacle_fcn < 0:
            result[nth_obstacle,0] = 1
        result[nth_obstacle,1] = -obstacle_fcn/math.pow(pos_and_size[nth_obstacle,2],2)
        result[nth_obstacle,2] = nth_obstacle
    
    return result

def StateCost(state, state_dot, control, inn_fcn, out_fcn, obstacles):
    x = state[0]
    y = state[1]
    phi = state[2]
    v0  = state[3]
    vd = control[0]
    wd = control[1]

    cost = 0

    out_f = out_fcn[0,0]*y+out_fcn[0,1]*x+out_fcn[0,2]
    out_g = out_fcn[1,0]*y+out_fcn[1,1]*x+out_fcn[1,2]
    out_h = out_fcn[2,0]*y+out_fcn[2,1]*x+out_fcn[2,2]
    out_i = out_fcn[3,0]*y+out_fcn[3,1]*x+out_fcn[3,2]
    
    inn_f = inn_fcn[0,0]*y+inn_fcn[0,1]*x+inn_fcn[0,2]
    inn_g = inn_fcn[1,0]*y+inn_fcn[1,1]*x+inn_fcn[1,2]
    inn_h = inn_fcn[2,0]*y+inn_fcn[2,1]*x+inn_fcn[2,2]
    inn_i = inn_fcn[3,0]*y+inn_fcn[3,1]*x+inn_fcn[3,2]

    if (out_f>0 and out_g<0 and out_h<0 and out_i>0) and not (inn_f>0 and inn_g<0 and inn_h<0 and inn_i>0):
        cost += 0
    else:
        cost += OFF_ROAD_P
        # print('fff')
    # print(x, y)
    # print(out_f, out_g, out_h, out_i, inn_f, inn_g, inn_h, inn_i)
    distance = DistanceFromTrack(inn_f, inn_g, inn_h, inn_i, inn_fcn)
    cost = cost + distance/(ROAD_WIDTH/2)*TRACK_ERR_P

    collision = ObstacleCollision(state, obstacles)
    obs_shape, _ = collision.shape
    for obs in range(obs_shape):
        if collision[obs,0] == 1:
            cost += abs(collision[obs,1])*COLLISION_P

    return cost
    