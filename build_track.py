import numpy as np
import math
'''
            g
          ______ 
         /     /
      h /     / f
       /     /    
       ------
         i
f,g intersection: [x1,y1]
g,h intersection: [x2,y2]
h,i intersection: [x3,y3]
i,f intersection: [x4,y4]                      
'''
def GetIntersection(l1, l2):
    a1 = l1[1]
    b1 = l1[0]
    c1 = l1[2]
    a2 = l2[1]
    b2 = l2[0]
    c2 = l2[2]
    x0 = (b1*c2-b2*c1)/(a1*b2-a2*b1)
    y0 = (c1*a2-c2*a1)/(a1*b2-a2*b1)
    pos = [x0,y0]
    return pos


def BuildTrack(mid_fcn, ROAD_WIDTH):
    mid_m = -mid_fcn[:,1]
    outer_fcn = np.copy(mid_fcn)
    outer_fcn[:,2] += np.array([ (ROAD_WIDTH/2)/math.cos(math.atan(mid_m[0])), 
                                -(ROAD_WIDTH/2)/math.cos(math.atan(mid_m[1])),
                                -(ROAD_WIDTH/2)/math.cos(math.atan(mid_m[2])),
                                 (ROAD_WIDTH/2)/math.cos(math.atan(mid_m[3]))])

    inner_fcn = np.copy(mid_fcn)
    inner_fcn[:,2] += np.array([-(ROAD_WIDTH/2)/math.cos(math.atan(mid_m[0])), 
                                 (ROAD_WIDTH/2)/math.cos(math.atan(mid_m[1])),
                                 (ROAD_WIDTH/2)/math.cos(math.atan(mid_m[2])),
                                -(ROAD_WIDTH/2)/math.cos(math.atan(mid_m[3]))])

    mid_intersec = np.array([GetIntersection(mid_fcn[0,:],mid_fcn[1,:]),
                             GetIntersection(mid_fcn[1,:],mid_fcn[2,:]),
                             GetIntersection(mid_fcn[2,:],mid_fcn[3,:]),
                             GetIntersection(mid_fcn[3,:],mid_fcn[0,:]),])

    inner_intersec = np.array([GetIntersection(inner_fcn[0,:],inner_fcn[1,:]),
                               GetIntersection(inner_fcn[1,:],inner_fcn[2,:]),
                               GetIntersection(inner_fcn[2,:],inner_fcn[3,:]),
                               GetIntersection(inner_fcn[3,:],inner_fcn[0,:]),])

    outer_intersec = np.array([GetIntersection(outer_fcn[0,:],outer_fcn[1,:]),
                               GetIntersection(outer_fcn[1,:],outer_fcn[2,:]),
                               GetIntersection(outer_fcn[2,:],outer_fcn[3,:]),
                               GetIntersection(outer_fcn[3,:],outer_fcn[0,:]),])

    return inner_fcn, outer_fcn, mid_intersec, inner_intersec, outer_intersec
    
