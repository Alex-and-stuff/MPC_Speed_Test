import numpy as np
from parameters import *
from numba import cuda
# a = np.zeros((4,3))
# x0 = np.array([x,y,phi,v0])
# print(a)
# a[:,0] = x0
# print(a)
# b = np.copy(a)
# b[:,1] = np.array([2,2,2,2])
# print(a)
# print(b)

# c = np.array([1,2,3,4])
# d = np.array([[1],[2],[3],[4]])
# print(c,c.shape)
# print(d,d.shape)
# a[:,0] = c
# print(a)
# print(a*0)


# s      = np.zeros((K, ))
# a =0
# for kth_rollout in range(K):
#     a+=1
#     s[kth_rollout] = a
# print(s)

# for i in range(2):
#     u_pert = np.random.normal(PERT_U, PERT_STDEV, size = (CONTROLS, N_HRZ-1))
#     print(u_pert)

# a = np.array([[1,2,3],
#             [4,5,6]])   

# print(a)
# max = np.max(a)
# b = np.exp(a/2)

# print(b)

cuda.detect()