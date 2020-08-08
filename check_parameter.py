import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import sys, os
import pdb
import glob
from scipy import linalg
from numpy import linalg as LA
#from liegroups import SO3
from pathlib import Path

def rotationError(pose_error):
    a = pose_error[0,0]
    b = pose_error[1][1]
    c = pose_error[2][2]
    d = 0.5*(a+b+c-1.0)
    return np.arccos(max(min(d,1.0),-1.0))


def translationError(pose_error):
    dx = pose_error[0][3]
    dy = pose_error[1][3]
    dz = pose_error[2][3]
    return np.sqrt(dx*dx+dy*dy+dz*dz)


startframe = 0
totalframe = 2

init_ell = 0.51

zero1 = np.array([0,0,0,1.0])

    
for init_ell in range(51, 52):
    # initial list to plot
    t_error_list = []
    r_error_list = [] 
    first_init_ell = init_ell / 100
    for seq in ['00', '01', '02', '03', '04', '05', '06', '07', '08', '09', '10']:
        filename = "results/lidar_geometric_result/cvo_geometric_"+seq+"_edge_detection_firstinitiell_"+str(first_init_ell)+".txt"
        groundtruthfile = "ground_truth/"+seq+".txt"
        for frame in range(startframe+1, totalframe-startframe):
            transform = np.genfromtxt(filename)
            groundtruth = np.genfromtxt(groundtruthfile)
            gt_now = groundtruth[frame,:].reshape((3,4))
            gt_now = (np.vstack([gt_now, zero1]))
            gt_next = groundtruth[frame+1,:].reshape((3,4))
            gt_next = np.vstack([gt_next, zero1])
            gt_curr = LA.inv(gt_now) @ gt_next
            
            # compute error
            curr_tf = transform[frame, :].reshape((3,4))
            curr_tf = np.vstack([curr_tf, zero1])
            error = linalg.inv(gt_curr) @ curr_tf
            r_err = rotationError(error)
            t_err = LA.norm(curr_tf[:3, 3] - gt_curr[:3, 3])

            # add to plot list
            t_error_list.append(t_err)
            r_error_list.append(r_err)

    # to numpy array!
    t_error_list = np.array(t_error_list)
    r_error_list = np.array(r_error_list)

    plt.plot(t_error_list, linestyle='solid', color='b')
    plt.plot(r_error_list, linestyle='dashed', color='g')
    plt.title('t-error and r-error for the first frame for every sequence with init_ell %.2f' % first_init_ell)
    plt.xlabel('sequence')
    plt.ylabel('error')
    plt.legend(['t-error (m)', 'r-error (rad)'])
    plt.show()
