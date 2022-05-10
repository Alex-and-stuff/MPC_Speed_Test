from ctypes.wintypes import tagRECT
from pickletools import UP_TO_NEWLINE
import numpy as np
from parameters import *
from build_track import *
from rollout_prediction import *
import time
import concurrent.futures
from numba import cuda


def main():
    x   = 150.91
    y   = 126.71
    phi = -0.5
    v0  = 2

    u      = np.zeros((CONTROLS, N_HRZ-1))
    u_pert = np.zeros((CONTROLS, N_HRZ-1))
    u_all  = np.zeros((CONTROLS, N_HRZ-1, K))
    u_opt  = np.zeros((CONTROLS, N_HRZ-1))
    pert_u = 0

    x_hrz  = np.zeros((STATES, N_HRZ, K))
    x_log  = np.zeros((STATES, N))
    u_log  = np.zeros((CONTROLS, N))
    s      = np.zeros((K, ))
    t_log  = np.zeros((1, N))

    x0 = np.array([x,y,phi,v0])
    x_log[:,0] = x0
    for k in range(N_HRZ-1):
        u[:,k] = np.array([2,0])

    inner_fcn, outer_fcn, mid_intersec, inner_intersec, outer_intersec = BuildTrack(mid_fcn, ROAD_WIDTH) 
    print("xlog")
    print(x_log)
    n_gen  = 0

    

    for curr_step in range(0,N-1):
        n_curr = math.ceil(curr_step*DT/T_STP)

        start_time = time.time()

        if n_gen < n_curr:
            
            np.roll(u,-1)
            u[:,-1] = np.array([0,0])
            x0 = x_log[:,n_gen]
            n_gen += 1
            x_hrz = 0*x_hrz
            s = 0*s

            
            ''' Original method '''
            for kth_rollout in range(K):
                hrz_k, s_k, u_k = RolloutPrediction(u, x0, inner_fcn, outer_fcn, obstacle)
                x_hrz[:,:,kth_rollout] = hrz_k
                s[kth_rollout] = s_k
                u_all[:,:,kth_rollout] = u_k
            

            ''' Second method multiprocessing'''
            # timer = 0
            # with concurrent.futures.ProcessPoolExecutor() as executor:
            #     result = [executor.submit(RolloutPrediction, u, x0, inner_fcn, outer_fcn, obstacle) for _ in range(K)]

            #     for kth_rollout in concurrent.futures.as_completed(result):
            #         # print(kth_rollout.result()[0])
                    
            #         # print(kth_rollout)
            #         x_hrz[:,:,timer] = kth_rollout.result()[0]#hrz_k
            #         s[timer] = kth_rollout.result()[1]#s_k
            #         u_all[:,:,timer] = kth_rollout.result()[2]#u_k
            #         timer += 1

            ''' Third method threading'''
            # timer = 0
            # with concurrent.futures.ThreadPoolExecutor() as executor:
            #     result = [executor.submit(RolloutPrediction, u, x0, inner_fcn, outer_fcn, obstacle) for _ in range(K)]

            #     for kth_rollout in concurrent.futures.as_completed(result):
            #         # print(kth_rollout.result()[0])
                    
            #         # print(kth_rollout)
            #         x_hrz[:,:,timer] = kth_rollout.result()[0]#hrz_k
            #         s[timer] = kth_rollout.result()[1]#s_k
            #         u_all[:,:,timer] = kth_rollout.result()[2]#u_k
            #         timer += 1

            rho = s.min()
            eta = np.exp(-1/LAMBDA*(s-rho))
            w = eta/np.sum(eta)

            u_opt = 0*u_opt
            for j in range(K):
                u_opt += w[j]*u_all[:,:,j]
            u = u_opt
            
        u_next = u_opt[:,0]
        x_dot = BicycleModelMPPI(x_log[:,curr_step],u_next)
        x_log[:,curr_step+1] = x_log[:,curr_step] + x_dot*DT
        u_log[:,curr_step]   = u_next
        t_log[0,curr_step]   = curr_step

        state = x_log[:, curr_step]
        control = u_next

        end_time = time.time()
        print(f"Runtime of one prediction is {end_time - start_time}(s)")
        
 
if __name__ == '__main__':
    main()


