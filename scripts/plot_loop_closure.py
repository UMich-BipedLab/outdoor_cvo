import sys, os
import numpy as np
import matplotlib.pyplot as plt

def gen_ip_table(ip_file_name, total_frames):
    table = np.zeros((total_frames, total_frames), dtype=float)
    with open(ip_file_name) as f:
        lines = f.readlines()
        for line in lines:
            ip_pair = line.split()
            f1 = int(ip_pair[0])
            f2 = int(ip_pair[1])
            ip = float(ip_pair[2])
            table[f1][f2] = ip
    return table

def gen_path(traj_file_name):
    pass

def plot_ip_table_single_frame(ip_table, frame_id):
    pass



if __name__ == "__main__":
    ip_file_name = sys.argv[1]
    traj_file_name = sys.argv[2]
    total_frames = int(sys.argv[3])
    frame_id = int( sys.argv[4])


    ip_table = gen_ip_table(ip_file_name, total_frames)
    plot_ip_table_single_frame(ip_table, frame_id)
    
