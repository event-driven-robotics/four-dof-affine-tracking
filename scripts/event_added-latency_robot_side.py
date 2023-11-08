 
import os
import numpy as np
from tqdm import tqdm
import matplotlib 
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, AutoMinorLocator
from scipy import interpolate

params = {'xtick.labelsize': 24,
          'ytick.labelsize': 24,
          'font.size': 13,
          'figure.autolayout': True,  # similar to tight_layout
          'figure.figsize': [6.4, 4.8],  # ratio 1.6 is pleasant
          'axes.titlesize': 13,
          'axes.labelsize': 28,
          'lines.linewidth': 2,
          'lines.markersize': 4,
          'legend.fontsize': 16}

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.style.use(params)

coords = []

def onclick(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata
    print (f'x = {ix}, y = {iy}')

    global coords
    coords.append((ix, iy))
    
    if len(coords) == 2:
        fig.canvas.mpl_disconnect(cid)

    return coords


files_list = []

tracker_event_path = "/usr/local/src/affine2dtracking/results"

for f in os.listdir(tracker_event_path):
    files_list.append(f)

sorted_files = sorted(files_list)

############################### event camera trans x (tracker side) ########################################################

# for f in sorted_files:
#     if ".txt" in f:
#         event_exp1_data = np.loadtxt(os.path.join(tracker_event_path, f), delimiter=" ", skiprows=2)[:,:11] 
#         delta_u = event_exp1_data[:,3]-320

############################### event camera trans x (robot side) ########################################################

file1 = 'ee_pose.csv'
file2 = 'modified.csv'

files_list = []
directories_list = []

event_error_list = []

event_exp0_path = "/usr/local/src/affine2dtracking/bag_files/event-camera-trans-x-added-latency"

for d in os.listdir(event_exp0_path):
    directories_list.append(d)

sorted_dir = sorted(directories_list)

open("time_to_failure.txt", "w").close()

for d in sorted_dir:
    event_exp0_ee_pose = np.loadtxt(os.path.join(event_exp0_path, d, file1), delimiter=",", skiprows=1)[:,:8] 
    ee_time = event_exp0_ee_pose[:, 0]-event_exp0_ee_pose[0, 0]
    ee_pose = event_exp0_ee_pose[:,2]

    event_exp0_data = np.loadtxt(os.path.join(event_exp0_path, d, file2), delimiter=",", skiprows=1)[:,:16] 
    timestamp = event_exp0_data[:,0]- event_exp0_data[0,0]
    delta_u = event_exp0_data[:,2]
    delta_v = event_exp0_data[:,3]
    delta_scale = event_exp0_data[:,4]
    delta_theta = event_exp0_data[:,5]

    start = next(x for x, val in enumerate(delta_u) if val != 0)
    start_value = timestamp[start]

    timestamp = timestamp[start:-1]
    timestamp = timestamp[:]-timestamp[0]
    delta_u = delta_u[start:-1]

    if (timestamp[-1]>10):
        end = next(x for x, val in enumerate(timestamp) if val > 10)

        timestamp = timestamp[0:end]
        delta_u = delta_u[0:end]

    if ee_time[0]>start_value: 
        index_start_ee = next(x for x, val in enumerate(ee_time) if val > start_value)

        ee_time = ee_time[index_start_ee:-1]
        ee_pose = ee_pose[index_start_ee:-1]
        ee_time = ee_time - ee_time[0]
    if (ee_time[-1]>10):
        index_end_ee = next(x for x, val in enumerate(ee_time) if val > 10)
        ee_time = ee_time[0:index_end_ee]
        ee_pose = ee_pose[0:index_end_ee]
    
    # subplot with all info 

    # coords.append((0,0))

    fig, ax = plt.subplots(3,1)
    cid = fig.canvas.mpl_connect('button_press_event', onclick)

    plt.suptitle(d + "-> gain = "+ str(event_exp0_data[0,11])+ ", filter value = "+  str(event_exp0_data[0,15]))


    ax[0].plot(ee_time, ee_pose, label='y ee')
    ax[1].plot(timestamp, delta_u+320, label='u')
    ax[2].plot(timestamp, delta_u, label='delta u')

    ax[0].set(ylabel="Position [m]")
    ax[1].set(ylabel="Position [pix]")
    ax[2].set(xlabel="Time [s]", ylabel="Error [pix]")

    plt.show()

    with open('time_to_failure_robot_side.txt', 'a') as file:
        if coords[0][0] != None:
            if coords[0][0] > 10:
                mean_error = np.mean(abs(delta_u)) 
                std_error = np.std(abs(delta_u))
                file.write(d + " " + "NO FAILURE, mean="+ str(mean_error) + ", std="+ str(std_error)+ "\n")   
            else:
                file.write(d + " " + str(coords[0][0])+"\n")

    coords = []